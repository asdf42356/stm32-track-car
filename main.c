#include "stm32f10x.h"
#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include <stdio.h>

/* ================================================================
   Shared Hardware Constants
   ================================================================ */
#define GYRO_SCALE      131.0f
#define CALIB_SAMPLES   160
#define KALM_PRECONV    40

/* ---- Kalman filter ---- */
#define Q_THETA     0.01f
#define Q_BIAS      0.001f
#define R_BIAS      5.0f
#define ENC_THRES   5
#define INNOV_THRES 3.0f

/* ---- Yaw/Encoder PID ---- */
#define KP_YAW  12.0f
#define KI_YAW  0.3f
#define KP_ENC  1.5f
#define I_MAX   100.0f

/* ---- 8ch line sensor (74HC4051) ---- */
#define LINE_OUT_PORT   GPIOA
#define LINE_OUT_PIN    GPIO_Pin_11
#define LINE_A0_PORT    GPIOB
#define LINE_A0_PIN     GPIO_Pin_13
#define LINE_A1_PORT    GPIOB
#define LINE_A1_PIN     GPIO_Pin_12
#define LINE_A2_PORT    GPIOB
#define LINE_A2_PIN     GPIO_Pin_14

/* ================================================================
   Task Configuration
   ================================================================ */
typedef struct {
    uint16_t base_speed;
    uint16_t min_speed;
    int16_t  left_trim;
    float    yaw_trim;
    float    kp_line, ki_line, kd_line;
    float    line_i_max;
    uint8_t  deb_w2b, deb_b2w;
    uint8_t  loop_ms;
    uint8_t  lap_target;
    float    diag_ca, diag_db;
    uint16_t search_speed;
} TaskConfig;

static const TaskConfig cfg[4] = {
    /* T1: straight A→B, stop on any black */
    { 500, 200,  0, 35.0f,    0, 0, 0,     0, 0, 0, 100, 0,    0,    0,   0 },
    /* T2: A→B→弧→C→D→弧→A, 1 lap */
    { 500, 200, 60, 40.0f,  120, 3, 20,  600, 3, 5, 100, 0,    0,    0,   0 },
    /* T3: A→C→弧→B→D→弧→A, 1 lap */
    { 500, 200, 60, 30.0f,  120, 3, 20,  600, 3, 5,  50, 0,    0,    0,   0 },
    /* T4: 3 laps, high speed, arc search */
    { 700, 200, 60, 30.0f,  160, 3, 10,  600, 2, 3,  30, 3, -126, 109, 300 },
};

/* ================================================================
   Shared State
   ================================================================ */
static float K_yaw, K_bias;
static float P00, P01, P11;
static float gyro_offset_z;

/* ---- Kalman ---- */
static void Kalman_Init(void)
{
    K_yaw  = 0.0f;
    K_bias = 0.0f;
    P00 = 1.0f;  P01 = 0.0f;  P11 = 1.0f;
}

static void Kalman_Predict(float gz, float dt)
{
    K_yaw += (gz - K_bias) * dt;
    float p00 = P00 - 2.0f * dt * P01 + dt * dt * P11 + Q_THETA;
    float p01 = P01 - dt * P11;
    float p11 = P11 + Q_BIAS;
    P00 = p00;  P01 = p01;  P11 = p11;
}

static void Kalman_UpdateBias(float gz)
{
    float innov = gz - K_bias;
    float S     = P11 + R_BIAS;
    float K0    = P01 / S;
    float K1    = P11 / S;
    K_yaw  += K0 * innov;
    K_bias += K1 * innov;
    P00 -= K0 * P01;
    P01 -= K0 * P11;
    P11 -= K1 * P11;
}

/* ---- USART ---- */
static void USART1_Init(void)
{
    GPIO_InitTypeDef g;
    USART_InitTypeDef u;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_9;
    g.GPIO_Mode  = GPIO_Mode_AF_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &g);
    g.GPIO_Pin  = GPIO_Pin_10;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &g);
    u.USART_BaudRate            = 115200;
    u.USART_WordLength          = USART_WordLength_8b;
    u.USART_StopBits            = USART_StopBits_1;
    u.USART_Parity              = USART_Parity_No;
    u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    u.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &u);
    USART_Cmd(USART1, ENABLE);
}

static void USART1_SendChar(char c)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, c);
}

static void USART1_SendString(const char *str)
{
    while (*str) USART1_SendChar(*str++);
}

/* ---- Line Sensor ---- */
static void LineSensor_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    g.GPIO_Pin  = LINE_OUT_PIN;
    g.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(LINE_OUT_PORT, &g);
    g.GPIO_Pin   = LINE_A0_PIN | LINE_A1_PIN | LINE_A2_PIN;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &g);
}

static void LineSensor_SelectCh(uint8_t ch)
{
    if (ch & 0x01) GPIO_SetBits(LINE_A0_PORT, LINE_A0_PIN);
    else           GPIO_ResetBits(LINE_A0_PORT, LINE_A0_PIN);
    if (ch & 0x02) GPIO_SetBits(LINE_A1_PORT, LINE_A1_PIN);
    else           GPIO_ResetBits(LINE_A1_PORT, LINE_A1_PIN);
    if (ch & 0x04) GPIO_SetBits(LINE_A2_PORT, LINE_A2_PIN);
    else           GPIO_ResetBits(LINE_A2_PORT, LINE_A2_PIN);
}

static uint8_t LineSensor_ReadCh(uint8_t ch)
{
    uint32_t i;
    LineSensor_SelectCh(ch);
    for (i = 0; i < 100; i++) __NOP();
    return GPIO_ReadInputDataBit(LINE_OUT_PORT, LINE_OUT_PIN) == Bit_SET ? 1 : 0;
}

static uint8_t LineSensor_AnyBlack(void)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
        if (LineSensor_ReadCh(i)) return 1;
    return 0;
}

static float LineSensor_GetPosition(void)
{
    uint8_t i, count = 0;
    float sum = 0;
    for (i = 0; i < 8; i++) {
        if (LineSensor_ReadCh(i)) { sum += (float)i; count++; }
    }
    return count ? sum / (float)count : -1.0f;
}

static uint8_t LineSensor_MidConsecutive(void)
{
    uint8_t c3 = LineSensor_ReadCh(3);
    uint8_t c4 = LineSensor_ReadCh(4);
    uint8_t c5 = LineSensor_ReadCh(5);
    uint8_t c6 = LineSensor_ReadCh(6);
    return ((c3 && c4) || (c4 && c5) || (c5 && c6));
}

/* ---- Point Alert: PB0 horn + PC13 LED (active-low) ---- */
static void PointAlert_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    g.GPIO_Pin   = GPIO_Pin_0;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &g);
    g.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOC, &g);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/* ---- Delay ---- */
static void Delay_ms(uint32_t ms)
{
    uint32_t i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 7200; j++);
}

static void PointAlert(void)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_0);
    GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    Delay_ms(300);
    GPIO_ResetBits(GPIOB, GPIO_Pin_0);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

/* ---- Buttons: PB3=T1, PB4=T2, PB5=T3, PA8=T4 ---- */
static void Button_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    g.GPIO_Mode  = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Pin   = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOB, &g);
    g.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &g);
}

static uint8_t Button_Read(void)
{
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == Bit_RESET) return 1;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == Bit_RESET) return 2;
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == Bit_RESET) return 3;
    if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET) return 4;
    return 0;
}

/* ---- Calibration ---- */
static void Calibrate(void)
{
    int32_t sum_z = 0;
    int16_t accel[3], gyro[3], temp;
    int i;
    float gz;
    USART1_SendString("Calibrating 2s, keep still...\r\n");
    for (i = 0; i < CALIB_SAMPLES; i++) {
        MPU6050_ReadData(accel, gyro, &temp);
        sum_z += gyro[2];
        Delay_ms(10);
    }
    gyro_offset_z = (float)sum_z / CALIB_SAMPLES;
    Kalman_Init();
    for (i = 0; i < KALM_PRECONV; i++) {
        MPU6050_ReadData(accel, gyro, &temp);
        gz = (gyro[2] - gyro_offset_z) / GYRO_SCALE;
        Kalman_Predict(gz, 0.01f);
        Kalman_UpdateBias(gz);
        Delay_ms(10);
    }
}

/* ================================================================
   Task 1: A→B straight, stop on any black line
   ================================================================ */
static void Task1_Run(const TaskConfig *c)
{
    int16_t accel[3], gyro[3], tmp;
    int16_t enc_l, enc_r, enc_diff;
    float gz, yaw_err, yaw_corr, enc_corr, yaw_integral;
    int16_t correction, speed_l, speed_r;
    char buf[80];
    float dt = c->loop_ms * 0.001f;

    USART1_SendString("Task 1: A->B straight\r\n");
    K_yaw += c->yaw_trim;
    yaw_integral = 0.0f;
    Motor_Enable();

    while (1) {
        MPU6050_ReadData(accel, gyro, &tmp);
        enc_l = Encoder_GetLeft();
        enc_r = Encoder_GetRight();
        gz = (gyro[2] - gyro_offset_z) / GYRO_SCALE;
        Kalman_Predict(gz, dt);
        enc_diff = enc_l - enc_r - c->left_trim;
        if (enc_diff > -ENC_THRES && enc_diff < ENC_THRES) {
            float innov = gz - K_bias;
            if (innov > -INNOV_THRES && innov < INNOV_THRES)
                Kalman_UpdateBias(gz);
        }
        yaw_err = K_yaw;
        if (yaw_err > 0.5f || yaw_err < -0.5f) {
            yaw_integral += yaw_err * dt;
            if (yaw_integral >  I_MAX) yaw_integral =  I_MAX;
            if (yaw_integral < -I_MAX) yaw_integral = -I_MAX;
        } else {
            yaw_integral = 0.0f;
        }
        yaw_corr = KP_YAW * yaw_err + KI_YAW * yaw_integral;
        enc_corr = KP_ENC * (float)enc_diff;
        correction = (int16_t)(enc_corr - yaw_corr);
        speed_l = c->base_speed - correction;
        speed_r = c->base_speed + correction;

        if (speed_l < 0) speed_l = 0;
        if (speed_l > 999) speed_l = 999;
        if (speed_r < 0) speed_r = 0;
        if (speed_r > 999) speed_r = 999;
        if (speed_l > 0 && speed_l < c->min_speed) speed_l = c->min_speed;
        if (speed_r > 0 && speed_r < c->min_speed) speed_r = c->min_speed;

        if (LineSensor_AnyBlack()) {
            Motor_Disable();
            USART1_SendString("*** BLACK LINE - STOP ***\r\n");
            PointAlert();
            while (1);
        }
        Motor_SetLeft(speed_l);
        Motor_SetRight(speed_r);

        sprintf(buf, "Y:%5.1f B:%5.2f L:%4d R:%4d\r\n",
                K_yaw, K_bias, speed_l, speed_r);
        USART1_SendString(buf);
        Delay_ms(c->loop_ms);
    }
}

/* ================================================================
   Task 2: A→B→弧→C→D→弧→A, 1 lap
   ================================================================ */
static void Task2_Run(const TaskConfig *c)
{
    int16_t accel[3], gyro[3], tmp;
    int16_t enc_l, enc_r, enc_diff;
    float gz, yaw_err, yaw_corr, enc_corr, yaw_integral;
    float line_pos, line_integral, last_line_err, target_yaw;
    uint8_t line_mode, black_count, debounce_cnt;
    int16_t correction, speed_l, speed_r;
    char buf[80];
    float dt = c->loop_ms * 0.001f;

    USART1_SendString("Task 2: A->B->arc->C->D->arc->A\r\n");
    K_yaw += c->yaw_trim;
    target_yaw = 0.0f;
    yaw_integral = 0.0f;
    line_integral = 0.0f;
    last_line_err = 0.0f;
    line_mode = 0;
    black_count = 0;
    debounce_cnt = 0;
    Motor_Enable();

    while (1) {
        MPU6050_ReadData(accel, gyro, &tmp);
        enc_l = Encoder_GetLeft();
        enc_r = Encoder_GetRight();
        gz = (gyro[2] - gyro_offset_z) / GYRO_SCALE;
        line_pos = LineSensor_GetPosition();

        if (line_mode) {
            if (line_pos >= 0.0f) {
                debounce_cnt = 0;
                float line_err = line_pos - 3.5f;
                line_integral += line_err * dt;
                if (line_integral >  c->line_i_max) line_integral =  c->line_i_max;
                if (line_integral < -c->line_i_max) line_integral = -c->line_i_max;
                float line_deriv = (line_err - last_line_err) / dt;
                last_line_err = line_err;
                correction = -(int16_t)(c->kp_line * line_err + c->ki_line * line_integral + c->kd_line * line_deriv);
                Kalman_Predict(gz, dt);
            } else {
                debounce_cnt++;
                if (debounce_cnt >= c->deb_b2w) {
                    black_count++;
                    if (black_count >= 2) {
                        Motor_Disable();
                        USART1_SendString("*** STOP ***\r\n");
                        PointAlert();
                        while (1);
                    }
                    target_yaw = (K_yaw > 0.0f) ? 385.0f : -385.0f;
                    yaw_integral = 0.0f;
                    line_integral = 0.0f;
                    last_line_err = 0.0f;
                    debounce_cnt = 0;
                    line_mode = 0;
                    correction = 0;
                    PointAlert();
                }
                Kalman_Predict(gz, dt);
            }
        } else {
            if (line_pos >= 0.0f) {
                debounce_cnt++;
                if (debounce_cnt >= c->deb_w2b) {
                    line_integral = 0.0f;
                    last_line_err = line_pos - 3.5f;
                    debounce_cnt = 0;
                    line_mode = 1;
                    correction = 0;
                    PointAlert();
                } else {
                    enc_diff = enc_l - enc_r - c->left_trim;
                    if (enc_diff > -ENC_THRES && enc_diff < ENC_THRES) {
                        float innov = gz - K_bias;
                        if (innov > -INNOV_THRES && innov < INNOV_THRES)
                            Kalman_UpdateBias(gz);
                    }
                    yaw_err = K_yaw - target_yaw;
                    if (yaw_err > 0.5f || yaw_err < -0.5f) {
                        yaw_integral += yaw_err * dt;
                        if (yaw_integral >  I_MAX) yaw_integral =  I_MAX;
                        if (yaw_integral < -I_MAX) yaw_integral = -I_MAX;
                    } else { yaw_integral = 0.0f; }
                    yaw_corr = KP_YAW * yaw_err + KI_YAW * yaw_integral;
                    enc_corr = KP_ENC * (float)enc_diff;
                    correction = (int16_t)(enc_corr - yaw_corr);
                }
                Kalman_Predict(gz, dt);
            } else {
                debounce_cnt = 0;
                Kalman_Predict(gz, dt);
                enc_diff = enc_l - enc_r - c->left_trim;
                if (enc_diff > -ENC_THRES && enc_diff < ENC_THRES) {
                    float innov = gz - K_bias;
                    if (innov > -INNOV_THRES && innov < INNOV_THRES)
                        Kalman_UpdateBias(gz);
                }
                yaw_err = K_yaw - target_yaw;
                if (yaw_err > 0.5f || yaw_err < -0.5f) {
                    yaw_integral += yaw_err * dt;
                    if (yaw_integral >  I_MAX) yaw_integral =  I_MAX;
                    if (yaw_integral < -I_MAX) yaw_integral = -I_MAX;
                } else { yaw_integral = 0.0f; }
                yaw_corr = KP_YAW * yaw_err + KI_YAW * yaw_integral;
                enc_corr = KP_ENC * (float)enc_diff;
                correction = (int16_t)(enc_corr - yaw_corr);
            }
        }

        speed_l = c->base_speed - correction;
        speed_r = c->base_speed + correction;
        if (speed_l < 0) speed_l = 0;
        if (speed_l > 999) speed_l = 999;
        if (speed_r < 0) speed_r = 0;
        if (speed_r > 999) speed_r = 999;
        if (speed_l > 0 && speed_l < c->min_speed) speed_l = c->min_speed;
        if (speed_r > 0 && speed_r < c->min_speed) speed_r = c->min_speed;
        Motor_SetLeft(speed_l);
        Motor_SetRight(speed_r);

        sprintf(buf, "Y:%5.1f BC:%d LM:%d LP:%2.1f L:%4d R:%4d\r\n",
                K_yaw, black_count, line_mode, line_pos, speed_l, speed_r);
        USART1_SendString(buf);
        Delay_ms(c->loop_ms);
    }
}

/* ================================================================
   Task 3: A→C→弧→B→D→弧→A, 1 lap, delayed stop
   ================================================================ */
static void Task3_Run(const TaskConfig *c)
{
    int16_t accel[3], gyro[3], tmp;
    int16_t enc_l, enc_r, enc_diff;
    float gz, yaw_err, yaw_corr, enc_corr, yaw_integral;
    float line_pos, line_integral, last_line_err, target_yaw;
    uint8_t line_mode, debounce_cnt;
    uint16_t loop_count;
    int16_t correction, speed_l, speed_r;
    char buf[80];
    float dt = c->loop_ms * 0.001f;

    USART1_SendString("Task 3: A->C->arc->B->D->arc->A\r\n");
    K_yaw += c->yaw_trim;
    target_yaw = 0.0f;
    yaw_integral = 0.0f;
    line_integral = 0.0f;
    last_line_err = 0.0f;
    line_mode = 0;
    debounce_cnt = 0;
    loop_count = 0;
    Motor_Enable();

    while (1) {
        MPU6050_ReadData(accel, gyro, &tmp);
        enc_l = Encoder_GetLeft();
        enc_r = Encoder_GetRight();
        gz = (gyro[2] - gyro_offset_z) / GYRO_SCALE;
        line_pos = LineSensor_GetPosition();

        if (line_mode) {
            if (line_pos >= 0.0f) {
                debounce_cnt = 0;
                float line_err = line_pos - 3.5f;
                line_integral += line_err * dt;
                if (line_integral >  c->line_i_max) line_integral =  c->line_i_max;
                if (line_integral < -c->line_i_max) line_integral = -c->line_i_max;
                float line_deriv = (line_err - last_line_err) / dt;
                last_line_err = line_err;
                correction = -(int16_t)(c->kp_line * line_err + c->ki_line * line_integral + c->kd_line * line_deriv);
                Kalman_Predict(gz, dt);
            } else {
                debounce_cnt++;
                if (debounce_cnt >= c->deb_b2w) {
                    target_yaw = (K_yaw > 0.0f) ? 512.0f : -450.0f;
                    yaw_integral = 0.0f;
                    line_integral = 0.0f;
                    last_line_err = 0.0f;
                    debounce_cnt = 0;
                    line_mode = 0;
                    correction = 0;
                    PointAlert();
                }
                Kalman_Predict(gz, dt);
            }
        } else {
            if (line_pos >= 0.0f) {
                debounce_cnt++;
                if (debounce_cnt >= c->deb_w2b) {
                    if (loop_count < 80) {
                        line_integral = 0.0f;
                        last_line_err = line_pos - 3.5f;
                        debounce_cnt = 0;
                        line_mode = 1;
                        correction = 0;
                        PointAlert();
                    } else {
                        Motor_Disable();
                        USART1_SendString("*** STOP ***\r\n");
                        PointAlert();
                        while (1);
                    }
                } else {
                    enc_diff = enc_l - enc_r - c->left_trim;
                    if (enc_diff > -ENC_THRES && enc_diff < ENC_THRES) {
                        float innov = gz - K_bias;
                        if (innov > -INNOV_THRES && innov < INNOV_THRES)
                            Kalman_UpdateBias(gz);
                    }
                    yaw_err = K_yaw - target_yaw;
                    if (yaw_err > 0.5f || yaw_err < -0.5f) {
                        yaw_integral += yaw_err * dt;
                        if (yaw_integral >  I_MAX) yaw_integral =  I_MAX;
                        if (yaw_integral < -I_MAX) yaw_integral = -I_MAX;
                    } else { yaw_integral = 0.0f; }
                    yaw_corr = KP_YAW * yaw_err + KI_YAW * yaw_integral;
                    enc_corr = KP_ENC * (float)enc_diff;
                    correction = (int16_t)(enc_corr - yaw_corr);
                }
                Kalman_Predict(gz, dt);
            } else {
                debounce_cnt = 0;
                Kalman_Predict(gz, dt);
                enc_diff = enc_l - enc_r - c->left_trim;
                if (enc_diff > -ENC_THRES && enc_diff < ENC_THRES) {
                    float innov = gz - K_bias;
                    if (innov > -INNOV_THRES && innov < INNOV_THRES)
                        Kalman_UpdateBias(gz);
                }
                yaw_err = K_yaw - target_yaw;
                if (yaw_err > 0.5f || yaw_err < -0.5f) {
                    yaw_integral += yaw_err * dt;
                    if (yaw_integral >  I_MAX) yaw_integral =  I_MAX;
                    if (yaw_integral < -I_MAX) yaw_integral = -I_MAX;
                } else { yaw_integral = 0.0f; }
                yaw_corr = KP_YAW * yaw_err + KI_YAW * yaw_integral;
                enc_corr = KP_ENC * (float)enc_diff;
                correction = (int16_t)(enc_corr - yaw_corr);
            }
        }

        speed_l = c->base_speed - correction;
        speed_r = c->base_speed + correction;
        if (speed_l < 0) speed_l = 0;
        if (speed_l > 999) speed_l = 999;
        if (speed_r < 0) speed_r = 0;
        if (speed_r > 999) speed_r = 999;
        if (speed_l > 0 && speed_l < c->min_speed) speed_l = c->min_speed;
        if (speed_r > 0 && speed_r < c->min_speed) speed_r = c->min_speed;
        Motor_SetLeft(speed_l);
        Motor_SetRight(speed_r);

        sprintf(buf, "Y:%5.1f LC:%d LM:%d LP:%2.1f L:%4d R:%4d\r\n",
                K_yaw, loop_count, line_mode, line_pos, speed_l, speed_r);
        USART1_SendString(buf);
        loop_count++;
        Delay_ms(c->loop_ms);
    }
}

/* ================================================================
   Task 4: 3 laps, arc-search entry, high speed
   ================================================================ */
static void Task4_Run(const TaskConfig *c)
{
    int16_t accel[3], gyro[3], tmp;
    int16_t enc_l, enc_r, enc_diff;
    float gz, yaw_err, yaw_corr, enc_corr, yaw_integral;
    float line_pos, line_integral, last_line_err, target_yaw;
    uint8_t line_mode, debounce_cnt;
    uint8_t lap_count, arc_id, search_mode;
    int16_t correction, speed_l, speed_r;
    char buf[80];
    float dt = c->loop_ms * 0.001f;

    USART1_SendString("Task 4: 3 laps high speed\r\n");
    K_yaw += c->yaw_trim;
    target_yaw = 0.0f;
    yaw_integral = 0.0f;
    line_integral = 0.0f;
    last_line_err = 0.0f;
    line_mode = 0;
    debounce_cnt = 0;
    lap_count = 0;
    arc_id = 0;
    search_mode = 0;
    Motor_Enable();

    while (1) {
        MPU6050_ReadData(accel, gyro, &tmp);
        enc_l = Encoder_GetLeft();
        enc_r = Encoder_GetRight();
        gz = (gyro[2] - gyro_offset_z) / GYRO_SCALE;
        line_pos = LineSensor_GetPosition();

        if (search_mode) {
            if (search_mode == 1) {
                Motor_SetLeft(-(int16_t)c->search_speed);
                Motor_SetRight((int16_t)c->search_speed);
            } else {
                Motor_SetLeft((int16_t)c->search_speed);
                Motor_SetRight(-(int16_t)c->search_speed);
            }
            Kalman_Predict(gz, dt);
            if (LineSensor_MidConsecutive()) {
                search_mode = 0;
                line_mode = 1;
                line_integral = 0.0f;
                last_line_err = LineSensor_GetPosition() - 3.5f;
                correction = 0;
                PointAlert();
            } else {
                goto task4_tail;
            }
        }

        if (line_mode) {
            if (line_pos >= 0.0f) {
                debounce_cnt = 0;
                float line_err = line_pos - 3.5f;
                line_integral += line_err * dt;
                if (line_integral >  c->line_i_max) line_integral =  c->line_i_max;
                if (line_integral < -c->line_i_max) line_integral = -c->line_i_max;
                float line_deriv = (line_err - last_line_err) / dt;
                last_line_err = line_err;
                correction = -(int16_t)(c->kp_line * line_err + c->ki_line * line_integral + c->kd_line * line_deriv);
                Kalman_Predict(gz, dt);
            } else {
                correction = 0;
                debounce_cnt++;
                if (debounce_cnt >= c->deb_b2w) {
                    if (arc_id == 0) {
                        target_yaw = K_yaw + c->diag_ca;
                        arc_id = 1;
                    } else {
                        target_yaw = K_yaw + c->diag_db;
                        arc_id = 0;
                        lap_count++;
                    }
                    yaw_integral = 0.0f;
                    line_integral = 0.0f;
                    last_line_err = 0.0f;
                    debounce_cnt = 0;
                    line_mode = 0;
                    correction = 0;
                    PointAlert();
                }
                Kalman_Predict(gz, dt);
            }
        } else {
            if (line_pos >= 0.0f) {
                debounce_cnt++;
                if (debounce_cnt >= c->deb_w2b) {
                    debounce_cnt = 0;
                    correction = 0;
                    PointAlert();
                    if (lap_count == 0 && arc_id == 0) {
                        line_integral = 0.0f;
                        last_line_err = line_pos - 3.5f;
                        line_mode = 1;
                    } else if (arc_id == 1) {
                        search_mode = 1;
                        Motor_SetLeft(-(int16_t)c->search_speed);
                        Motor_SetRight((int16_t)c->search_speed);
                        goto task4_tail;
                    } else {
                        if (lap_count >= c->lap_target) {
                            Motor_Disable();
                            USART1_SendString("*** 3 LAPS DONE ***\r\n");
                            PointAlert();
                            while (1);
                        }
                        search_mode = 2;
                        Motor_SetLeft((int16_t)c->search_speed);
                        Motor_SetRight(-(int16_t)c->search_speed);
                        goto task4_tail;
                    }
                } else {
                    enc_diff = enc_l - enc_r - c->left_trim;
                    if (enc_diff > -ENC_THRES && enc_diff < ENC_THRES) {
                        float innov = gz - K_bias;
                        if (innov > -INNOV_THRES && innov < INNOV_THRES)
                            Kalman_UpdateBias(gz);
                    }
                    yaw_err = K_yaw - target_yaw;
                    if (yaw_err > 0.5f || yaw_err < -0.5f) {
                        yaw_integral += yaw_err * dt;
                        if (yaw_integral >  I_MAX) yaw_integral =  I_MAX;
                        if (yaw_integral < -I_MAX) yaw_integral = -I_MAX;
                    } else { yaw_integral = 0.0f; }
                    yaw_corr = KP_YAW * yaw_err + KI_YAW * yaw_integral;
                    enc_corr = KP_ENC * (float)enc_diff;
                    correction = (int16_t)(enc_corr - yaw_corr);
                }
                Kalman_Predict(gz, dt);
            } else {
                Kalman_Predict(gz, dt);
                enc_diff = enc_l - enc_r - c->left_trim;
                if (enc_diff > -ENC_THRES && enc_diff < ENC_THRES) {
                    float innov = gz - K_bias;
                    if (innov > -INNOV_THRES && innov < INNOV_THRES)
                        Kalman_UpdateBias(gz);
                }
                yaw_err = K_yaw - target_yaw;
                if (yaw_err > 0.5f || yaw_err < -0.5f) {
                    yaw_integral += yaw_err * dt;
                    if (yaw_integral >  I_MAX) yaw_integral =  I_MAX;
                    if (yaw_integral < -I_MAX) yaw_integral = -I_MAX;
                } else { yaw_integral = 0.0f; }
                yaw_corr = KP_YAW * yaw_err + KI_YAW * yaw_integral;
                enc_corr = KP_ENC * (float)enc_diff;
                correction = (int16_t)(enc_corr - yaw_corr);
            }
        }

        speed_l = c->base_speed - correction;
        speed_r = c->base_speed + correction;
        if (speed_l < 0) speed_l = 0;
        if (speed_l > 999) speed_l = 999;
        if (speed_r < 0) speed_r = 0;
        if (speed_r > 999) speed_r = 999;
        if (speed_l > 0 && speed_l < c->min_speed) speed_l = c->min_speed;
        if (speed_r > 0 && speed_r < c->min_speed) speed_r = c->min_speed;
        Motor_SetLeft(speed_l);
        Motor_SetRight(speed_r);

        sprintf(buf, "Y:%5.1f Lp:%d Aid:%d Sm:%d LP:%2.1f L:%4d R:%4d\r\n",
                K_yaw, lap_count, arc_id, search_mode, line_pos, speed_l, speed_r);
        USART1_SendString(buf);
    task4_tail:
        Delay_ms(c->loop_ms);
    }
}

/* ================================================================
   Main
   ================================================================ */
int main(void)
{
    uint8_t task;

    /* ---- Shared peripheral init ---- */
    USART1_Init();
    USART1_SendString("Starting...\r\n");
    Button_Init();
    LineSensor_Init();
    PointAlert_Init();
    Motor_Init();
    Encoder_Init();

    MPU6050_Init();
    Delay_ms(100);

    USART1_SendString("Press: PB3=T1 PB4=T2 PB5=T3 PA8=T4\r\n");
    task = 0;
    while (!task) {
        uint8_t btn = Button_Read();
        if (btn) {
            Delay_ms(20);
            if (Button_Read() == btn) {
                task = btn;
                {
                    char buf[20];
                    sprintf(buf, "Task %d\r\n", task);
                    USART1_SendString(buf);
                }
            }
        }
    }
    PointAlert();
    Calibrate();

    if (task == 1)      Task1_Run(&cfg[0]);
    else if (task == 2) Task2_Run(&cfg[1]);
    else if (task == 3) Task3_Run(&cfg[2]);
    else if (task == 4) Task4_Run(&cfg[3]);

    while (1);
}

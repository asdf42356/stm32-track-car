#include "mpu6050.h"

#define I2C2_TIMEOUT  500000UL

/* ---- I2C2 bus recovery: bit-bang SCL to free stuck SDA after reset ---- */
static void I2C2_BusRecovery(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    volatile uint32_t d;
    uint8_t i;

    I2C_Cmd(I2C2, DISABLE);

    /* PB10=SCL, PB11=SDA: both open-drain output for manual recovery */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Release SDA first */
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    for (d = 0; d < 5000; d++);

    /* 9 SCL pulses to complete any partial slave transfer and free SDA */
    for (i = 0; i < 9; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        for (d = 0; d < 5000; d++);
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        for (d = 0; d < 5000; d++);
    }

    /* STOP condition: SDA low -> SCL high -> SDA high */
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    for (d = 0; d < 5000; d++);
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    for (d = 0; d < 5000; d++);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    for (d = 0; d < 5000; d++);
}

/* Timeout-protected I2C event wait. Returns 0 on timeout, 1 on success. */
static uint8_t I2C2_WaitEvent(uint32_t event)
{
    uint32_t timeout = I2C2_TIMEOUT;
    while (!I2C_CheckEvent(I2C2, event)) {
        if (--timeout == 0) return 0;
    }
    return 1;
}

static void I2C2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* Recover potentially stuck bus (e.g. after watchdog reset) before init */
    I2C2_BusRecovery();

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Mode              = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle         = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1       = 0x00;
    I2C_InitStructure.I2C_Ack               = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed        = 400000;
    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);
}

void MPU6050_WriteByte(uint8_t reg, uint8_t data)
{
    I2C_GenerateSTART(I2C2, ENABLE);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return;
    I2C_Send7bitAddress(I2C2, MPU6050_ADDR, I2C_Direction_Transmitter);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
    I2C_SendData(I2C2, reg);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    I2C_SendData(I2C2, data);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;
    I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t MPU6050_ReadByte(uint8_t reg)
{
    uint8_t val = 0;

    I2C_GenerateSTART(I2C2, ENABLE);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0;
    I2C_Send7bitAddress(I2C2, MPU6050_ADDR, I2C_Direction_Transmitter);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return 0;
    I2C_SendData(I2C2, reg);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return 0;

    I2C_GenerateSTART(I2C2, ENABLE);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return 0;
    I2C_Send7bitAddress(I2C2, MPU6050_ADDR, I2C_Direction_Receiver);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return 0;
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) return 0;
    val = I2C_ReceiveData(I2C2);
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    return val;
}

void MPU6050_ReadData(int16_t *accel, int16_t *gyro, int16_t *temp)
{
    uint8_t buf[14];
    uint8_t i;

    I2C_GenerateSTART(I2C2, ENABLE);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return;
    I2C_Send7bitAddress(I2C2, MPU6050_ADDR, I2C_Direction_Transmitter);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) return;
    I2C_SendData(I2C2, MPU6050_ACCEL_XOUT_H);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED)) return;

    I2C_GenerateSTART(I2C2, ENABLE);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT)) return;
    I2C_Send7bitAddress(I2C2, MPU6050_ADDR, I2C_Direction_Receiver);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) return;

    for (i = 0; i < 13; i++) {
        if (!I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) return;
        buf[i] = I2C_ReceiveData(I2C2);
    }
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    if (!I2C2_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED)) return;
    buf[13] = I2C_ReceiveData(I2C2);
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);

    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    *temp    = (int16_t)((buf[6] << 8) | buf[7]);
    gyro[0]  = (int16_t)((buf[8] << 8) | buf[9]);
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);
}

void MPU6050_Init(void)
{
    I2C2_Init();

    MPU6050_WriteByte(MPU6050_PWR_MGMT_1,   0x00);  /* Wake up */
    MPU6050_WriteByte(MPU6050_SMPLRT_DIV,   0x07);  /* 1kHz / 8 = 125Hz */
    MPU6050_WriteByte(MPU6050_CONFIG,       0x00);  /* No DLPF */
    MPU6050_WriteByte(MPU6050_GYRO_CONFIG,  0x00);  /* +/-250 deg/s */
    MPU6050_WriteByte(MPU6050_ACCEL_CONFIG, 0x00);  /* +/-2g */
}

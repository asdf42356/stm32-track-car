#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

#define MPU6050_ADDR          0xD0

/* MPU6050 Register Map */
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_TEMP_OUT_H    0x41
#define MPU6050_GYRO_XOUT_H   0x43
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_WHO_AM_I      0x75

void MPU6050_Init(void);
uint8_t MPU6050_ReadByte(uint8_t reg);
void MPU6050_WriteByte(uint8_t reg, uint8_t data);
void MPU6050_ReadData(int16_t *accel, int16_t *gyro, int16_t *temp);

#endif

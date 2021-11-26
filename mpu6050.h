/*
 * mpu6050.h
 *
 *  Created on: Nov 26, 2021
 *      Author: grkm
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"

#define mpu6050           0x68
#define mpu6050addr       mpu6050 << 1
#define whoAmIReg         0x75 // to read 0x68 is exist or not
#define powerManagmentReg 0x6B
#define sampleRateDiv 	  0x19 // sampleRate = gyroRate / (1 + sampleDiv)
#define gyroConf		  0x1B
#define accelConf		  0x1C

#define accelMeasure      0x3B
#define gyroMeasure       0x43


#define validCondition1 (whoAreYou == mpu6050)

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

uint8_t whoAreYou;
uint8_t MemData;

typedef enum{
	degS250  = 0,
	degS500  = 1,
	degS1000 = 2,
	degS2000 = 3
}gyroScale_t;

typedef enum{
	g2  = 0,
	g4  = 1,
	g8  = 2,
	g16 = 3
}accelScale_t;

int16_t RAWgyroX;
int16_t RAWgyroY;
int16_t RAWgyroZ;

int16_t RAWaccelX;
int16_t RAWaccelY;
int16_t RAWaccelZ;

float Ax, Ay, Az;
float Gx, Gy, Gz;

void mpu6050Config(void);
void mpu6050Init(void);
void mpu6050powerOn(void);
void mpu6050Sampling(void);
void mpu6050GyroScale(gyroScale_t scale);
void mpu6050AccelScale(accelScale_t scale);

void mpu6050GyroRead(void);
void mpu6050AccelRead(void);



#endif /* INC_MPU6050_H_ */

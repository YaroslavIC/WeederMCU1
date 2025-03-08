/*
 * ITG3200.c
 *
 *  Created on: Mar 8, 2025
 *      Author: HeroPC
 *      based https://github.com/kounst/QuadVolucer/tree/master
 */

#include "ITG3200.h"

float GyroXNeutral=0;
float GyroYNeutral=0;
float GyroZNeutral=0;


void writemem(I2C_HandleTypeDef  hi2cX,unsigned char i2c_address, unsigned char reg_address, uint8_t value )
{
	uint8_t i2c_tx[2];

	i2c_tx[0] = reg_address;
	i2c_tx[1] = value;
	HAL_I2C_Master_Transmit(&hi2cX, i2c_address , (uint8_t*) i2c_tx, 2, 500);
}

void readmem (I2C_HandleTypeDef  hi2cX,unsigned char i2c_address,unsigned char addr, uint8_t *buf, int n)
{
	uint8_t i2c_tx[1];

	i2c_tx[0] = addr;

	HAL_I2C_Master_Transmit(&hi2cX, ITG3200_SLAVE_ADDR, (uint8_t*) i2c_tx, 1, 500);
	HAL_I2C_Master_Receive(&hi2cX, ITG3200_SLAVE_ADDR, (uint8_t*) buf, n, 100);
}

void InitGyro(I2C_HandleTypeDef  hi2cX)
{
  writemem (hi2cX,ITG3200_SLAVE_ADDR, 0x15, ITG3200_SMPLRT_DIV); // 0 = 8 kHz
  writemem (hi2cX,ITG3200_SLAVE_ADDR, 0x16, ITG3200_FS_SEL + ITG3200_DLPF_CFG);  // 2000 G/s + 42Hz
  writemem (hi2cX,ITG3200_SLAVE_ADDR, 0x3E, ITG3200_PWR_MANAG);

}

void ReadGyro(I2C_HandleTypeDef  hi2cX, float *ValX, float *ValY, float *ValZ){

	uint8_t buf[6];

	readmem(hi2cX,ITG3200_SLAVE_ADDR,29,buf,6);

    *ValX = ((float)((int16_t)(buf[0] << 8) | buf[1])) / 14.375 - GyroXNeutral;
    *ValY = ((float)((int16_t)(buf[2] << 8) | buf[3])) / 14.375 - GyroYNeutral;
    *ValZ = ((float)((int16_t)(buf[4] << 8) | buf[5])) / 14.375 - GyroZNeutral;


}

void Calibration_Gyro(I2C_HandleTypeDef hi2cX) {

	int i;
	float sneutralX = 0;
	float sneutralY = 0;
	float sneutralZ = 0;
	float neutralX = 0;
	float neutralY = 0;
	float neutralZ = 0;

	for (i = 0; i < 100; i++) {
		ReadGyro(hi2cX, &neutralX, &neutralY, &neutralZ);
		sneutralX += neutralX;
		sneutralY += neutralY;
		sneutralZ += neutralZ;
		HAL_Delay(20);
	}

	GyroXNeutral = sneutralX / 100;
	GyroYNeutral = sneutralY / 100;
	GyroZNeutral = sneutralZ / 100;

}






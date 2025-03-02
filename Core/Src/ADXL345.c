/**
 **********************************************************************************
 * @file   ADXL345.c
 * @author Hossein.M (https://github.com/Hossein-M98)
 * @brief  ADXL345 driver
 *         Functionalities of the this file:
 *          + Full feature
 *          + Support for I2C communication protocol
 *          + Easy to port
 **********************************************************************************
 *
 * Copyright (c) 2021 Mahda Embedded System (MIT License)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN Activity OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************
 */

/* Includes ---------------------------------------------------------------------*/
#include "ADXL345.h"
#include <string.h>
#include "math.h"
struct AdxlCommands
{
	uint8_t	DEVID;	/*	Device ID	*/
	uint8_t	THRESH_TAP;	/*	Tap threshold	*/
	uint8_t	OFSX;	/*	X-axis offset	*/
	uint8_t	OFSY;	/*	Y-axis offset	*/
	uint8_t	OFSZ;	/*	Z-axis offset	*/
	uint8_t	DUR;	/*	Tap duration	*/
	uint8_t	Latent;	/*	Tap latency	*/
	uint8_t	Window;	/*	Tap window	*/
	uint8_t	THRESH_ACT;	/*	Activity threshold	*/
	uint8_t	THRESH_INACT;	/*	Inactivity threshold	*/
	uint8_t	TIME_INACT;	/*	Inactivity time	*/
	uint8_t	ACT_INACT_CTL;	/*	Axis enable control for activity and inactivity detection	*/
	uint8_t	THRESH_FF;	/*	Free-fall threshold	*/
	uint8_t	TIME_FF;	/*	Free-fall time	*/
	uint8_t	TAP_AXES;	/*	Axis control for single tap/double tap	*/
	uint8_t	ACT_TAP_STATUS;	/*	Source of single tap/double tap	*/
	uint8_t	BW_RATE;	/*	Data rate and power mode control	*/
	uint8_t	POWER_CTL;	/*	Power-saving features control	*/
	uint8_t	INT_ENABLE;	/*	Interrupt enable control	*/
	uint8_t	INT_MAP;	/*	Interrupt mapping control	*/
	uint8_t	INT_SOURCE;	/*	Source of interrupts	*/
	uint8_t	DATA_FORMAT;	/*	Data format control	*/
	uint8_t	DATAX0;	/*	X-Axis Data 0	*/
	uint8_t	DATAX1;	/*	X-Axis Data 1	*/
	uint8_t	DATAY0;	/*	Y-Axis Data 0	*/
	uint8_t	DATAY1;	/*	Y-Axis Data 1	*/
	uint8_t	DATAZ0;	/*	Z-Axis Data 0	*/
	uint8_t	DATAZ1;	/*	Z-Axis Data 1	*/
	uint8_t	FIFO_CTL;	/*	FIFO control	*/
	uint8_t	FIFO_STATUS;	/*	FIFO status	*/
};
struct AdxlCommands AdxlReg = {0x00, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22,
	0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D,
	0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};

enum AdxlBitNum
{
	D0,
	D1,
	D2,
	D3,
	D4,
	D5,
	D6,
	D7
};

struct Adxl_Data
{
	uint8_t id;
	int16_t x, y, z;
	uint8_t raw[6];
	uint8_t int_src;
}Adxl345;

int ADXL_MaxG;

double adxl_scale;

void ADXL344_SetMaxG(I2C_HandleTypeDef  hi2cX,int MaxG)
{
	uint8_t i2c_tx[2];
	i2c_tx[0] = AdxlReg.DATA_FORMAT;
	ADXL_MaxG = MaxG;

	switch (MaxG) {
	  case 2:
  	    i2c_tx[1] = ((0 << D3) | (0 << D1) | (0 << D0));
        break;
	  case 4:
	    i2c_tx[1] = ((0 << D3) | (0 << D1) | (1 << D0));
        break;
	  case 8:
	    i2c_tx[1] = ((0 << D3) | (1 << D1) | (0 << D0));
        break;
	  case 16:
	    i2c_tx[1] = ((0 << D3) | (1 << D1) | (1 << D0));
        break;
	}
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*)i2c_tx, 2, 1000);

}
void ADXL345_Conf(I2C_HandleTypeDef  hi2cX){
	uint8_t i2c_tx[2];
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*)&AdxlReg.DEVID , 1, 1000);
	HAL_Delay(100);
	HAL_I2C_Master_Receive(&hi2cX, ADXL_ADDR, (uint8_t*)&Adxl345.id, 1, 1000);

	i2c_tx[0] = AdxlReg.BW_RATE;
	i2c_tx[1] = ((1 << D3) | (1 << D2)); // 400 Hz 1100
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*)i2c_tx, 2, 1000);

	i2c_tx[0] = AdxlReg.DATA_FORMAT;
	//i2c_tx[1] = ((1 << D1) | (1 << D0));  // 11  16g
	i2c_tx[1] = ((1 << D1) | (1 << D0));
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*)i2c_tx, 2, 1000);

	i2c_tx[0] = AdxlReg.INT_MAP;
	i2c_tx[1] = (1 << D7);
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*)i2c_tx, 2, 1000);

	i2c_tx[0] = AdxlReg.INT_ENABLE;
	i2c_tx[1] = (1 << D7);
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*)i2c_tx, 2, 1000);

	i2c_tx[0] = AdxlReg.POWER_CTL;
	i2c_tx[1] = (1 << D3);
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*)i2c_tx, 2, 1000);
}

void ADXL345_Read(I2C_HandleTypeDef hi2cX, int16_t *ValX, int16_t *ValY,
		int16_t *ValZ) {
	uint8_t i2c_tx[2];

	i2c_tx[0] = AdxlReg.DATAX0;
	i2c_tx[1] = 0x06;
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*) i2c_tx, 2, 500);
	HAL_I2C_Master_Receive(&hi2cX, ADXL_ADDR, (uint8_t*) Adxl345.raw, 6, 100);

	*ValX = (int16_t)((int16_t) (Adxl345.raw[1] << 8) | Adxl345.raw[0]);
	*ValY = (int16_t)((int16_t) (Adxl345.raw[3] << 8) | Adxl345.raw[2]);
	*ValZ = (int16_t)((int16_t) (Adxl345.raw[5] << 8) | Adxl345.raw[4]);

};

void ADXL345_Read_G(I2C_HandleTypeDef hi2cX, float *ValX, float *ValY,	float *ValZ)
{
	uint8_t i2c_tx[2];

	i2c_tx[0] = AdxlReg.DATAX0;
	i2c_tx[1] = 0x06;
	HAL_I2C_Master_Transmit(&hi2cX, ADXL_ADDR, (uint8_t*) i2c_tx, 2, 500);
	HAL_I2C_Master_Receive(&hi2cX, ADXL_ADDR, (uint8_t*) Adxl345.raw, 6, 100);

	*ValX = ((double)((int16_t) (Adxl345.raw[1] << 8) | Adxl345.raw[0]))/adxl_scale;
	*ValY = ((double)((int16_t) (Adxl345.raw[3] << 8) | Adxl345.raw[2]))/adxl_scale;
	*ValZ = ((double)((int16_t) (Adxl345.raw[5] << 8) | Adxl345.raw[4]))/adxl_scale;

};


void ADXL345_Calibartion(I2C_HandleTypeDef hi2cX, uint16_t samples_count) {

	int16_t ValX = 0;
	int16_t ValY = 0;
	int16_t ValZ = 0;
	int32_t sValX = 0;
	int32_t sValY = 0;
	int32_t sValZ = 0;

	for (uint16_t i = 0; i < samples_count; i++) {
		ADXL345_Read(hi2cX, &ValX, &ValY, &ValZ);

		sValX = sValX + ValX;
		sValY = sValY + ValY;
		sValZ = sValZ + ValZ;

	}

	 adxl_scale = sqrt(
			pow((double) ((float) sValX / (float) samples_count), 2)
					+ pow((float) ((float) sValY / (float) samples_count), 2)
					+ pow((float) ((float) sValZ / (float) samples_count), 2));



}



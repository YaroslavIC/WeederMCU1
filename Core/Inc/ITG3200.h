/*
 * ITG3200.h
 *
 *  Created on: Mar 8, 2025
 *      Author: HeroPC
 *       based https://github.com/kounst/QuadVolucer/tree/master
 */

#ifndef INC_ITG3200_H_
#define INC_ITG3200_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"


#define ITG3200_SLAVE_ADDR  0xD0
#define ITG3200_DLPF_CFG    2      //register: DLPF_CFG - low pass filter configuration 98Hz
#define ITG3200_FS_SEL      0x18   //2000�/s
#define ITG3200_SMPLRT_DIV  0      //8000Hz: gyro sample rate
#define ITG3200_PWR_MANAG   0x03   //register: Power Management  --  value: PLL with Z Gyro reference



void InitGyro(I2C_HandleTypeDef  hi2cX);
void ReadGyro(I2C_HandleTypeDef  hi2cX, float *ValX, float *ValY, float *ValZ);
void Calibration_Gyro(I2C_HandleTypeDef  hi2cX);


#ifdef __cplusplus
}
#endif

#endif /* INC_ITG3200_H_ */

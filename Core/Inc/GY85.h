/*
 * GY85.h
 *
 *  Created on: Mar 01  2025
 *      Author: HeroPC
 */

#ifndef GY85_H
#define GY85_H


#ifdef __cplusplus
 extern "C" {
#endif



#include "stm32f4xx_hal.h"
#include "math.h"

#define HMC5883L_ADDRESS  0x1E

#define COMPASS_CONFIG_REGISTER_A 0x00
#define COMPASS_CONFIG_REGISTER_B 0x01
#define COMPASS_MODE_REGISTER     0x02
#define COMPASS_DATA_REGISTER     0x03

#define Data_Output_X_MSB 			0x03
#define Data_Output_X_LSB 			0x04
#define Data_Output_Z_MSB 			0x05
#define Data_Output_Z_LSB 			0x06
#define Data_Output_Y_MSB 			0x07
#define Data_Output_Y_LSB 			0x08

#define COMPASS_SAMPLE1 0x00 // 1-average(Default)
#define COMPASS_SAMPLE2 0x20 // 2-average
#define COMPASS_SAMPLE4 0x40 // 4-average
#define COMPASS_SAMPLE8 0x60 // 8-average

#define COMPASS_RATE0_75 0x00 // 0.75Hz
#define COMPASS_RATE1_5  0x04 // 1.5Hz
#define COMPASS_RATE3    0x08 // 3Hz
#define COMPASS_RATE7_5  0x0C // 7.5Hz
#define COMPASS_RATE15   0x10 // 15Hz (Default)
#define COMPASS_RATE30   0x14 // 30Hz
#define COMPASS_RATE75   0x18 // 75Hz

#define COMPASS_MEASURE_NORMAL   0x00 // Normal measurement configuration (Default)
#define COMPASS_MEASURE_POSITIVE 0x01 // Positive bias configuration for X, Y, and Z axes.
#define COMPASS_MEASURE_NEGATIVE 0x02 // Negative bias configuration for X, Y and Z axes.

#define COMPASS_SCALE_088  0x00  //0.88 Ga
#define COMPASS_SCALE_130  0x20 //1.3 Ga (Default)
#define COMPASS_SCALE_190  0x40 //1.9 Ga
#define COMPASS_SCALE_250  0x60 //2.5 Ga
#define COMPASS_SCALE_400  0x80 //4.0 Ga
#define COMPASS_SCALE_470  0xA0 //4.7 Ga
#define COMPASS_SCALE_560  0xC0 //5.6 Ga
#define COMPASS_SCALE_810  0xE0 //8.1 Ga

#define COMPASS_CONTINUOUS 0x00  //Continuous-Measurement Mode.
#define COMPASS_SINGLE     0x01  //Single-Measurement Mode (Default).
#define COMPASS_IDLE       0x02  //Idle Mode. Device is placed in idle mode




void compass_SetDeclination( int declination_degs , int declination_mins, char declination_dir );
void SetSamplingMode(I2C_HandleTypeDef hi2cX, uint16_t sampling_mode , uint16_t rate, uint16_t measure);
void SetScaleMode(I2C_HandleTypeDef hi2cX,uint8_t ScaleMode);
void SetMeasureMode(I2C_HandleTypeDef hi2cX, uint8_t Measure);
uint16_t CompasReadAxis(I2C_HandleTypeDef hi2cX,uint16_t reg);
float CompasRead6Axis(I2C_HandleTypeDef hi2cX);
float compensate(float compass_X, float compass_Y, float compass_Z, float pitch, float roll);

#ifdef __cplusplus
  }
#endif

#endif // GY_85_H

/*
 * AHRS.h
 *
 *  Created on: Mar 30, 2025
 *      Author: HeroPC
 */

#ifndef INC_AHRSroutines_H_
#define INC_AHRSroutines_H_

#ifdef __cplusplus
    extern "C" {
#endif


#include "stm32f4xx_hal.h"
#include <time.h>
#include <string.h>
#include "math.h"
#include "QMC5883L.h"
#include "ADXL345.h"
#include "stdio.h"
#include "../Fusion/Fusion.h"
#include "ITG3200.h"
#include <stdint.h>

#define AHRS_SAMPLE_RATE (10)

extern void AHRS_Full_Init(I2C_HandleTypeDef  hi2cX) ;
extern void AHRS_Calculation(I2C_HandleTypeDef  hi2cX);
extern void AHRS_Calculation_print(void) ;

#ifdef __cplusplus
    }
#endif


#endif /* INC_AHRSroutines_H_ */

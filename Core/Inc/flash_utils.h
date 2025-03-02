/*
 * flash_utils.h
 *
 *  Created on: Mar 2, 2025
 *      Author: HeroPC
 */

#ifndef INC_FLASH_UTILS_H_
#define INC_FLASH_UTILS_H_



#ifdef __cplusplus
    extern "C" {
#endif



#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "string.h"



#define STM32F401_ID_ADDR         0x1FFF7A10
#define STM32F401_EE_START_ADDR   0x08020000
#define STM32F401_EE_END_ADDR     0x0803FFFF

typedef struct
{
   uint32_t  PWMSpeedLength;
   float Left_PWM[20];
   float Left_Speed[20];
   float Right_PWM[20];
   float Right_Speed[20];


} SFlash_data_storage;

void FLASH_SaveSetting();
void FLASH_LoadSetting();

void FlashSaveUint32(uint32_t addr, uint32_t data);


#ifdef __cplusplus
    }
#endif

#endif /* INC_FLASH_UTILS_H_ */

/*
 * Wheel.h
 *
 *  Created on: Mar 3, 2025
 *      Author: HeroPC
 */

#ifndef INC_WHEEL_H_
#define INC_WHEEL_H_

#ifdef __cplusplus
    extern "C" {
#endif

#include "main.h"
#include "math.h"

#define I2C_AS5600					0x36
#define AS5600_RAW_ANGLE_H			0x0C
#define AS5600_RAW_ANGLE_L			0x0D
#define AS5600_ANGLE_H				0x0E
#define AS5600_ANGLE_L				0x0F

#define ADC_CHANNELS_NUM   			2
#define ADC_CHANNEL_LENGTH 			50

#define  MAX_ANGLE_WHEEL_ARRAY 20
enum DirectionEnum {WH_CW, WH_CCW,WH_STOP};
enum WheelSide {wsLeft,wsRight,wsNone};
#define MAX_COMPASS_ARRAY 50


class WheelData {
   private:
   	enum DirectionEnum Direction;

   	TIM_HandleTypeDef htim;
   	I2C_HandleTypeDef hi2c;

   	GPIO_TypeDef* GPIOx_INA;
   	uint16_t GPIO_Pin_INA;
   	GPIO_PinState PinState_INA;

   	GPIO_TypeDef* GPIOx_INB;
   	uint16_t GPIO_Pin_INB;
   	GPIO_PinState PinState_INB;

   	enum WheelSide ws;

   	float delta_angle;

   public:
   	WheelData(

   			I2C_HandleTypeDef hi2c_,
   			TIM_HandleTypeDef htim_,

   			uint16_t  PWM_Channel_,

   			GPIO_TypeDef* GPIOx_INA_,
   			uint16_t GPIO_Pin_INA_,
   			GPIO_PinState PinState_INA_,


   			GPIO_TypeDef* GPIOx_INB_,
   			uint16_t GPIO_Pin_INB_,
   			GPIO_PinState PinState_INB_,


   			enum WheelSide WS_);

   	void ReadAS5600_Curr(float curr_) ;
   	void Set_Speed(float Speed_, int PIDmode_);
   	void Calculation(void);

   	float Current_Speed, Target_Speed;
   	uint32_t time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY];
   	double angle[MAX_ANGLE_WHEEL_ARRAY];
   	float speed[MAX_ANGLE_WHEEL_ARRAY];
   	float Disired_Target_diff[MAX_ANGLE_WHEEL_ARRAY];
   	float curr[MAX_ANGLE_WHEEL_ARRAY];

   	float Derror ;


   	//float averspeed;	//,turns_left,prior_quadrant,current_quadrant;
   	uint32_t PWM_Channel;
   	float PWM_Value;
   	float PID_value_P, PID_value_I, PID_value_D;

   //	uint32_t speed_priortime;
   //	int32_t delta_PWM;
   	//float delta_speed;

   	float PID_P, PID_I, PID_D, PID_sum_I;

   	int PIDMode;

   	//	uint32_t PWM[31];

   };



#ifdef __cplusplus
    }
#endif



#endif /* INC_WHEEL_H_ */

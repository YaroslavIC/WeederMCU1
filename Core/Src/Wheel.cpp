/*
 * Wheel.c
 *
 *  Created on: Mar 3, 2025
 *      Author: HeroPC
 */


#include "Wheel.hpp"

 WheelData::WheelData(

		I2C_HandleTypeDef hi2c_,
		TIM_HandleTypeDef htim_,

		uint16_t    PWM_Channel_,

		GPIO_TypeDef *GPIOx_INA_,
		uint16_t GPIO_Pin_INA_,
		GPIO_PinState PinState_INA_,

		GPIO_TypeDef *GPIOx_INB_,
		uint16_t GPIO_Pin_INB_,
		GPIO_PinState PinState_INB_,

		WheelSide WS_)

{

	hi2c = hi2c_;

	htim = htim_;

	PWM_Channel = PWM_Channel_;

	GPIOx_INA = GPIOx_INA_;
	GPIO_Pin_INA = GPIO_Pin_INA_;
	PinState_INA = PinState_INA_;

	GPIOx_INB = GPIOx_INB_;
	GPIO_Pin_INB = GPIO_Pin_INB_;
	PinState_INB = PinState_INB_;

	ws = WS_;

	HAL_TIM_PWM_Start(&htim, PWM_Channel);
	__HAL_TIM_SET_COMPARE(&htim, PWM_Channel, 0);

	PID_P = 40;
	PID_I = 0;
	PID_D = 0;

	PID_Sum_I = 0;

	Target_Speed = 0;
	Current_Speed = 0;

	PWM_Value = 0;
   	OnOffCalculation = 1;

	Set_Speed(0, 0);

}

 void WheelData::ReadAS5600_Curr(float curr_) {
		uint8_t regData[2];

		HAL_I2C_Mem_Read(&hi2c, (I2C_AS5600 << 1), AS5600_ANGLE_H,
				I2C_MEMADD_SIZE_8BIT, (uint8_t*) &regData, 2, 0x10000);


		float tmpangle =  roundf(((float) (((uint16_t) regData[0] << 8
				| (uint16_t) regData[1]) & (uint16_t) 0xFFF)) / 4096 * 360);

		uint32_t tmpmsec = HAL_GetTick();


		delta_angle = angle[MAX_ANGLE_WHEEL_ARRAY - 1] - tmpangle;


		  tmpCurrent_Speed = 0;

	    if ((tmpmsec - time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1])>1) {

		 tmpCurrent_Speed = (
				(1000 * fabsf(delta_angle))
						/ (tmpmsec - time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1]))
				/ 360 * 60;
	    } else {
	    	 tmpCurrent_Speed = 0;
	    }

	    if (tmpCurrent_Speed<3) {tmpCurrent_Speed=0;};


		// сдвигаем в массиве все в сторону 0, в последнюю ячейку запишим новые данные
		for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
			angle[i - 1] = angle[i];
			time_ms_wheel[i - 1] = time_ms_wheel[i];
			speed[i - 1] = speed[i];
			Disired_Target_diff[i - 1] = Disired_Target_diff[i];
			curr[i - 1] = curr[i];
		};

		ss =0;

		for (uint8_t i = 0; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
		  ss = ss + speed[i];
		}

		aver_Current_Speed = ss / MAX_ANGLE_WHEEL_ARRAY;


		angle[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpangle;
		time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpmsec;
		speed[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpCurrent_Speed;

		if ((fabsf((speed[MAX_ANGLE_WHEEL_ARRAY - 1] - speed[MAX_ANGLE_WHEEL_ARRAY - 2]))>20) && (tmpCurrent_Speed>0.1)) {
			speed[MAX_ANGLE_WHEEL_ARRAY - 1] = speed[MAX_ANGLE_WHEEL_ARRAY - 2];
			tmpCurrent_Speed = speed[MAX_ANGLE_WHEEL_ARRAY - 1];
		}


		speed[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpCurrent_Speed;

		Disired_Target_diff[MAX_ANGLE_WHEEL_ARRAY - 1] = Target_Speed - tmpCurrent_Speed;

//		float tmpPID_sum_I = 0;
//		for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
//			tmpPID_sum_I = tmpPID_sum_I + Disired_Target_diff[i];
//		}
//		PID_sum_I = tmpPID_sum_I;

		Current_Speed = tmpCurrent_Speed;

		curr[MAX_ANGLE_WHEEL_ARRAY - 1] = curr_;




 }

/*

void WheelData::ReadAS5600_Curr2(float curr_) // pulling 0.5 ms
		{
	uint8_t regData[2];

	HAL_I2C_Mem_Read(&hi2c, (I2C_AS5600 << 1), AS5600_ANGLE_H,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) &regData, 2, 0x10000);


	float tmpangle =  roundf(((float) (((uint16_t) regData[0] << 8
			| (uint16_t) regData[1]) & (uint16_t) 0xFFF)) / 4096 * 360);

	uint32_t tmpmsec = HAL_GetTick();



	if ((Target_Speed > 0) || (OnOffCalculation==0)) {

		if (ws == wsLeft) {

			if (Direction == WH_CW) {

				if (tmpangle < angle[MAX_ANGLE_WHEEL_ARRAY - 1]) {
					delta_angle = (360 - angle[MAX_ANGLE_WHEEL_ARRAY - 1])
							+ tmpangle;
				} else {
					delta_angle = angle[MAX_ANGLE_WHEEL_ARRAY - 1] - tmpangle;
				}
			};


			if (Direction == WH_CCW) {

				if (tmpangle > angle[MAX_ANGLE_WHEEL_ARRAY - 1]) {
					delta_angle = angle[MAX_ANGLE_WHEEL_ARRAY - 1]
							+ (360 - tmpangle);
				} else {
					delta_angle = angle[MAX_ANGLE_WHEEL_ARRAY - 1] - tmpangle;
				}
			};

		};

		if (ws == wsRight) {

				if (Direction == WH_CW) {

					if (tmpangle > angle[MAX_ANGLE_WHEEL_ARRAY - 1]) {
						delta_angle = angle[MAX_ANGLE_WHEEL_ARRAY - 1]
								+ (360 - tmpangle);
					} else {
						delta_angle = angle[MAX_ANGLE_WHEEL_ARRAY - 1]
								- tmpangle;
					}
				};

				if (Direction == WH_CCW) {
					if (tmpangle < angle[MAX_ANGLE_WHEEL_ARRAY - 1]) {
						delta_angle = (360 - angle[MAX_ANGLE_WHEEL_ARRAY - 1])
								+ tmpangle;
					} else {
						delta_angle = angle[MAX_ANGLE_WHEEL_ARRAY - 1] - tmpangle;
					}

				}

			}

			if (Direction == WH_STOP) {
				tmpangle = angle[MAX_ANGLE_WHEEL_ARRAY - 1];
				delta_angle = 0;
			}

		} else {
			tmpangle = angle[MAX_ANGLE_WHEEL_ARRAY - 1];
			delta_angle = 0;
		};


	float tmpCurrent_Speed = 0;

    if ((tmpmsec - time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1])>1) {

	 tmpCurrent_Speed = (
			(1000 * fabsf(delta_angle))
					/ (tmpmsec - time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1]))
			/ 360 * 60;
    } else {
    	 tmpCurrent_Speed = 0;
    }



	float tmpCurrent_Speed = (
			(1000 * (tmpangle - angle[MAX_ANGLE_WHEEL_ARRAY - 1]))
					/ (tmpmsec - time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1]))
			/ 360 * 60;




	// сдвигаем в массиве все в сторону 0, в последнюю ячейку запишим новые данные
	for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
		angle[i - 1] = angle[i];
		time_ms_wheel[i - 1] = time_ms_wheel[i];
		speed[i - 1] = speed[i];
		Disired_Target_diff[i - 1] = Disired_Target_diff[i];
		curr[i - 1] = curr[i];
	};

    // текущая  скорость будт обновлена только если нет перехода угла через ноль

	if (((tmpangle-angle[MAX_ANGLE_WHEEL_ARRAY - 1])*(angle[MAX_ANGLE_WHEEL_ARRAY - 1]-angle[MAX_ANGLE_WHEEL_ARRAY - 2]))<0) {
		angle[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpangle;
		time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpmsec;
		curr[MAX_ANGLE_WHEEL_ARRAY - 1] = curr_;
		return;
	} else {
		angle[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpangle;
		time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpmsec;
		speed[MAX_ANGLE_WHEEL_ARRAY - 1] = tmpCurrent_Speed;

		Disired_Target_diff[MAX_ANGLE_WHEEL_ARRAY - 1] = Target_Speed - tmpCurrent_Speed;

		float tmpPID_sum_I = 0;
		for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
			tmpPID_sum_I = tmpPID_sum_I + Disired_Target_diff[i];
		}
		PID_sum_I = tmpPID_sum_I;

		Current_Speed = tmpCurrent_Speed;

		curr[MAX_ANGLE_WHEEL_ARRAY - 1] = curr_;

	}



}

*/


void WheelData::Set_Speed_Assistant(uint8_t start_cycles, uint32_t _PWM_Value) {

	if (start_cycles_counter == start_cycles) {

		start_cycles_sequence = 0;
		start_cycles_counter = 0;
	};

	if (start_cycles_sequence == 1) {

		start_cycles_counter++;
		__HAL_TIM_SET_COMPARE(&htim, PWM_Channel, (uint32_t ) _PWM_Value);

	};

	if ((old_speed_assistant == 0) && (Target_Speed != 0)
			&& (start_cycles_counter == 0)) {


		start_cycles_counter = 1;
		start_cycles_sequence = 1;

		__HAL_TIM_SET_COMPARE(&htim, PWM_Channel, (uint32_t ) _PWM_Value);
	};

	old_speed_assistant = Target_Speed;

}

void WheelData::Set_Speed(float Speed_, int PIDmode_)
{
    PIDMode = PIDmode_;

	Target_Speed = fabsf(Speed_);

	if (ws == wsLeft) {

		if (Speed_ > 0) {
			PinState_INA = GPIO_PIN_RESET;
			PinState_INB = GPIO_PIN_SET;
			Direction = WH_CCW;
		};
		if (Speed_ < 0) {
			PinState_INA = GPIO_PIN_SET;
			PinState_INB = GPIO_PIN_RESET;
			Direction = WH_CW;
		};

	}

	if (ws == wsRight) {

		if (Speed_  < 0) {
			PinState_INA = GPIO_PIN_RESET;
			PinState_INB = GPIO_PIN_SET;
			Direction = WH_CCW;
		};
		if (Speed_  > 0) {
			PinState_INA = GPIO_PIN_SET;
			PinState_INB = GPIO_PIN_RESET;
			Direction = WH_CW;
		};

	}

	if (Speed_  == 0) {
		PinState_INA = GPIO_PIN_RESET;
		PinState_INB = GPIO_PIN_RESET;
		Direction = WH_STOP;
		PID_Sum_I = 0;
		__HAL_TIM_SET_COMPARE(&htim, PWM_Channel, (uint32_t ) 0 );

		for (uint8_t i = 0; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
			angle[i] = 0;
			time_ms_wheel[i] = 0;
			speed[i] = 0;
			Disired_Target_diff[i] = 0;
			curr[i] = 0;
		};

	};


	HAL_GPIO_WritePin(GPIOx_INA, GPIO_Pin_INA, PinState_INA);
	HAL_GPIO_WritePin(GPIOx_INB, GPIO_Pin_INB, PinState_INB);

}

void WheelData::DirectControlDriver(GPIO_PinState _PinState_INA,GPIO_PinState _PinState_INB, uint32_t  PWM_Value)
{
	__HAL_TIM_SET_COMPARE(&htim, PWM_Channel, (uint32_t ) PWM_Value);
	HAL_GPIO_WritePin(GPIOx_INA, GPIO_Pin_INA, _PinState_INA);
	HAL_GPIO_WritePin(GPIOx_INB, GPIO_Pin_INB, _PinState_INB);

}

void WheelData::OnOff_Calculation(int OnOff)
{
	OnOffCalculation = OnOff;
}


void WheelData::Calculation(void)
{
	if (start_cycles_sequence == 1) {return;};

	if (OnOffCalculation==0) {return;};

	if (fabsf(Target_Speed) > 0) {

		PID_value_P = PID_P * (Target_Speed - (speed[MAX_ANGLE_WHEEL_ARRAY-1]+speed[MAX_ANGLE_WHEEL_ARRAY-2]+speed[MAX_ANGLE_WHEEL_ARRAY-3])/3);


		PID_Sum_I = PID_Sum_I + (Target_Speed - (speed[MAX_ANGLE_WHEEL_ARRAY-1]+speed[MAX_ANGLE_WHEEL_ARRAY-2]+speed[MAX_ANGLE_WHEEL_ARRAY-3])/3);

		PID_value_I = PID_I * PID_Sum_I;

	//	PID_value_I = PID_I * Disired_Target_diff[MAX_ANGLE_WHEEL_ARRAY];

		if ((time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY]
				- time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1]) > 0) {
			Derror = (Disired_Target_diff[MAX_ANGLE_WHEEL_ARRAY]
					- Disired_Target_diff[MAX_ANGLE_WHEEL_ARRAY - 1])
					/ (time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY]
							- time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1]);
		} else {
			Derror = 0;
		};

		PID_value_D = PID_D * Derror;

		PWM_Value = PWM_Value + PID_value_P + PID_value_I + PID_value_D;

		if (PWM_Value < 0) {
			PWM_Value = 0;
		};
		if (PWM_Value > 15000) {
			PWM_Value = 15000;
		};

	} else {

		PWM_Value = 0;

	}

	__HAL_TIM_SET_COMPARE(&htim, PWM_Channel, (uint32_t ) PWM_Value);
}


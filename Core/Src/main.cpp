/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cli_app.h"
#include "task.h"
#include <string.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define  MAX_ANGLE_WHEEL_ARRAY 20
enum DirectionEnum {WH_CW, WH_CCW,WH_STOP};
enum WheelSide {wsLeft,wsRight,wsNone};


//MPU9250_t mpu9250;
//uint8_t isDeviceConnected = 0;

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
	void Set_Speed(float Speed_);
	void Calculation(void);

	float Current_Speed, Target_Speed;
	uint32_t time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY];
	double angle[MAX_ANGLE_WHEEL_ARRAY];
	float speed[MAX_ANGLE_WHEEL_ARRAY];
	float Disired_Target_diff[MAX_ANGLE_WHEEL_ARRAY];
	float curr[MAX_ANGLE_WHEEL_ARRAY];

	//float averspeed;	//,turns_left,prior_quadrant,current_quadrant;
	uint32_t PWM_Channel;
	float PWM_Value, PID_value_P, PID_value_I;

//	uint32_t speed_priortime;
//	int32_t delta_PWM;
	//float delta_speed;

	float PID_P, PID_I, PID_D, PID_sum_I;

	//	uint32_t PWM[31];

};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_AS5600					0x36
#define AS5600_RAW_ANGLE_H			0x0C
#define AS5600_RAW_ANGLE_L			0x0D
#define AS5600_ANGLE_H				0x0E
#define AS5600_ANGLE_L				0x0F

#define ADC_CHANNELS_NUM   2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task1ms */
osThreadId_t Task1msHandle;
const osThreadAttr_t Task1ms_attributes = {
  .name = "Task1ms",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task10ms */
osThreadId_t Task10msHandle;
const osThreadAttr_t Task10ms_attributes = {
  .name = "Task10ms",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task100ms */
osThreadId_t Task100msHandle;
const osThreadAttr_t Task100ms_attributes = {
  .name = "Task100ms",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

extern osThreadId_t cmdLineTaskHandle;

uint16_t adcData[ADC_CHANNELS_NUM*10];
float adcVoltage[ADC_CHANNELS_NUM*10];


WheelData* clLeftW;
WheelData* clRightW;


float set_speed;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void Task1msHandler(void *argument);
void Task10msHandler(void *argument);
void Task100msHandler(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

	PID_P = 0.01;
	PID_I = 0;
	PID_D = 0;

	Target_Speed = 0;
	Current_Speed = 0;

}

void WheelData::ReadAS5600_Curr(float curr_) // pulling 0.5 ms
		{
	uint8_t regData[2];

	HAL_I2C_Mem_Read(&hi2c, (I2C_AS5600 << 1), AS5600_ANGLE_H,
			I2C_MEMADD_SIZE_8BIT, (uint8_t*) &regData, 2, 0x10000);

	float tmpangle = ((float) (((uint16_t) regData[0] << 8
			| (uint16_t) regData[1]) & (uint16_t) 0xFFF)) / 4096 * 360;

	uint32_t tmpmsec = HAL_GetTick();




	float tmpCurrent_Speed = fabsf(
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



void WheelData::Set_Speed(float Speed_) {

	Target_Speed = fabsf(Speed_);

	if (ws == wsLeft) {

		if (Speed_ > 0) {
			PinState_INA = GPIO_PIN_RESET;
			PinState_INB = GPIO_PIN_SET;
			Direction = WH_CW;
		};
		if (Speed_ < 0) {
			PinState_INA = GPIO_PIN_SET;
			PinState_INB = GPIO_PIN_RESET;
			Direction = WH_CCW;
		};

		if (Speed_  == 0) {
			PinState_INA = GPIO_PIN_RESET;
			PinState_INB = GPIO_PIN_RESET;
			Direction = WH_STOP;
		};
	}

	if (ws == wsRight) {

		if (Speed_  < 0) {
			PinState_INA = GPIO_PIN_RESET;
			PinState_INB = GPIO_PIN_SET;
			Direction = WH_CW;
		};
		if (Speed_  > 0) {
			PinState_INA = GPIO_PIN_SET;
			PinState_INB = GPIO_PIN_RESET;
			Direction = WH_CCW;
		};

		if (Speed_  == 0) {
			PinState_INA = GPIO_PIN_RESET;
			PinState_INB = GPIO_PIN_RESET;
			Direction = WH_STOP;
		};
	}

	HAL_GPIO_WritePin(GPIOx_INA, GPIO_Pin_INA, PinState_INA);
	HAL_GPIO_WritePin(GPIOx_INB, GPIO_Pin_INB, PinState_INB);

}


void WheelData::Calculation(void)
{
	if (fabsf(Target_Speed) > 0) {
		PID_value_P =  PID_P * (Target_Speed - Current_Speed);

		if (PID_value_P > 20)  { PID_value_P = 20;  };
		if (PID_value_P < -20) { PID_value_P = -20;	};

		PID_sum_I = 0;
		for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
			PID_sum_I = PID_sum_I + Disired_Target_diff[i];
		}

		PWM_Value = PWM_Value + PID_value_P  + PID_I * PID_sum_I;

		if (PWM_Value < 0)     {PWM_Value = 0;	};
		if (PWM_Value > 50000) {PWM_Value = 50000; };


	} else {

		PWM_Value = 0;

	}

	__HAL_TIM_SET_COMPARE(&htim, PWM_Channel,
			(uint32_t) PWM_Value);
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
    for (uint8_t i = 0; i < ADC_CHANNELS_NUM; i++)
    {
      adcVoltage[i] = adcData[i] * 3.3 / 4095;
    }
  }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */





  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }



 clLeftW = new WheelData(hi2c2, htim1,
	TIM_CHANNEL_1,
	GPIOB,
	R_INA_Pin, GPIO_PIN_RESET,
	GPIOB,
	R_INB_Pin, GPIO_PIN_RESET, wsLeft);

	clRightW = new WheelData(hi2c1, htim1,
	TIM_CHANNEL_4,
	GPIOA,
	L_INA_Pin, GPIO_PIN_RESET,
	GPIOA,
	L_INB_Pin, GPIO_PIN_RESET, wsRight);





  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task1ms */
  Task1msHandle = osThreadNew(Task1msHandler, NULL, &Task1ms_attributes);

  /* creation of Task10ms */
  Task10msHandle = osThreadNew(Task10msHandler, NULL, &Task10ms_attributes);

  /* creation of Task100ms */
  Task100msHandle = osThreadNew(Task100msHandler, NULL, &Task100ms_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {






    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1599;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUELED_GPIO_Port, BLUELED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R_INA_Pin|R_INB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L_INB_Pin|L_INA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUELED_Pin */
  GPIO_InitStruct.Pin = BLUELED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUELED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : R_INA_Pin R_INB_Pin */
  GPIO_InitStruct.Pin = R_INA_Pin|R_INB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : L_INB_Pin L_INA_Pin */
  GPIO_InitStruct.Pin = L_INB_Pin|L_INA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task1msHandler */
/**
* @brief Function implementing the Task1ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task1msHandler */
void Task1msHandler(void *argument)
{
  /* USER CODE BEGIN Task1msHandler */
  /* Infinite loop */
  for(;;)
  {
  }
  /* USER CODE END Task1msHandler */
}

/* USER CODE BEGIN Header_Task10msHandler */
/**
* @brief Function implementing the Task10ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task10msHandler */
void Task10msHandler(void *argument) {
	/* USER CODE BEGIN Task10msHandler */
	/* Infinite loop */
	for (;;) {

		clLeftW->Calculation();
		clRightW->Calculation();

	}
	/* USER CODE END Task10msHandler */
}

/* USER CODE BEGIN Header_Task100msHandler */
/**
* @brief Function implementing the Task100ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task100msHandler */
void Task100msHandler(void *argument) {
	/* USER CODE BEGIN Task100msHandler */
	/* Infinite loop */
	for (;;) {


		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, ADC_CHANNELS_NUM*10);

		clLeftW->ReadAS5600_Curr(0);
		clRightW->ReadAS5600_Curr(0);



	 	clLeftW->Set_Speed(set_speed);
	//	clRightW->Set_Speed(set_speed);

	}
	/* USER CODE END Task100msHandler */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if (htim->Instance == TIM2) {

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // мигаем светодиодом


	} // end of TIM2
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

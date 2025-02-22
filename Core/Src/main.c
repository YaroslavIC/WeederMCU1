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
#include "task.h"
#include <string.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define  MAX_ANGLE_WHEEL_ARRAY 10
enum DirectionEnum {WH_CW, WH_CCW,WH_STOP};
//MPU9250_t mpu9250;
uint8_t isDeviceConnected = 0;


struct WheelData
{
	enum DirectionEnum Direction;

	TIM_HandleTypeDef htim;
	I2C_HandleTypeDef hi2c;

	GPIO_TypeDef* GPIOx_INA;
	uint16_t GPIO_Pin_INA;
	GPIO_PinState PinState_INA;

	GPIO_TypeDef* GPIOx_INB;
	uint16_t GPIO_Pin_INB;
	GPIO_PinState PinState_INB;

	float Target_speed;
	uint32_t time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY];
	double angle[MAX_ANGLE_WHEEL_ARRAY];
	float speed[MAX_ANGLE_WHEEL_ARRAY];

	float averspeed,turns_left,prior_quadrant,current_quadrant;
	uint32_t PWM_Channel;
	 int32_t PWM_Value;
	uint32_t turns;

	uint32_t speed_priortime;
	int32_t delta_PWM;
	float delta_speed;

	float PID;
	uint32_t PWM[31];

};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_AS5600					0x36
#define AS5600_RAW_ANGLE_H			0x0C
#define AS5600_RAW_ANGLE_L			0x0D
#define AS5600_ANGLE_H				0x0E
#define AS5600_ANGLE_L				0x0F
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

UART_HandleTypeDef huart1;

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


uint32_t task1msCnt = 0;
uint32_t task10msCnt = 0;
uint32_t task100msCnt = 0;


/* USER CODE BEGIN PV */

struct WheelData LeftW,RightW;

//float angle_left[MAX_ANGLE_WHEEL_ARRAY],angle_right[MAX_ANGLE_WHEEL_ARRAY];
//float speed_left[MAX_ANGLE_WHEEL_ARRAY],speed_right[MAX_ANGLE_WHEEL_ARRAY];
//float averspeed_left , averspeed_right,turns_left,prior_quadrant_left,current_quadrant_left;

//uint32_t time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY];
//uint32_t aindex = 0;
uint8_t value_acc[6] ;
uint8_t value_mag[6] ;
float speedr;

uint32_t newval=0;



//uint32_t PWM_Channel_1,PWM_Channel_4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
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





float HAL_ReadAS5600_I2Cx(I2C_HandleTypeDef hi2cX) // pulling 0.5 ms
{
  uint8_t regData[2];
  HAL_I2C_Mem_Read(&hi2cX, (I2C_AS5600 << 1) , AS5600_ANGLE_H	, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&regData , 2, 0x10000);
  return ((float)(((uint16_t)regData[0]<<8 | (uint16_t)regData[1]) & (uint16_t)0xFFF))/4096*360;
}


/*

float HAL_LSM303D_I2C1(void){
	uint8_t regData[2];
    regData[0] = 0x20;
    regData[1] = 0x57; // 0101 0111 ;
    HAL_I2C_Master_Transmit(&hi2c1, ACC_I2C_ADDRESS, (uint8_t *)&regData,     2,10);

    regData[0] = 0x24;
    regData[1] = 0xEC; // 11101100
    HAL_I2C_Master_Transmit(&hi2c1, ACC_I2C_ADDRESS, (uint8_t *)&regData,     2,10);

    regData[0] = 0x25;
    regData[1] = 0x00; // 00000000
    HAL_I2C_Master_Transmit(&hi2c1, ACC_I2C_ADDRESS, (uint8_t *)&regData,     2,10);

    regData[0] = 0x26;
    regData[1] = 0x00; // 00000000;  |
    HAL_I2C_Master_Transmit(&hi2c1, ACC_I2C_ADDRESS, (uint8_t *)&regData,     2,10);

	HAL_I2C_Mem_Read(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_TEMP_OUT_H_M , I2C_MEMADD_SIZE_8BIT, (uint8_t *)&value_acc, 2, 1);

	return 0;
}
*/


void Calibration_Motor(void)
{
  for (int i=1;i<30;i++) {
	LeftW.Target_speed = i;
	RightW.Target_speed = i;
	HAL_Delay(20000);
	LeftW.PWM[i] = LeftW.PWM_Value;
	RightW.PWM[i] = RightW.PWM_Value;
  }

}
void Motor_Init(void)
{
	  LeftW.hi2c = hi2c2;
	  LeftW.htim = htim1;
	  LeftW.PWM_Channel = TIM_CHANNEL_1;
	  LeftW.PWM_Value = 0;
	  LeftW.GPIOx_INA = GPIOB;
	  LeftW.GPIOx_INB = GPIOB;
	  LeftW.GPIO_Pin_INA = R_INA_Pin;
	  LeftW.GPIO_Pin_INB = R_INB_Pin;
	  LeftW.PinState_INA    = GPIO_PIN_RESET;
	  LeftW.PinState_INB    = GPIO_PIN_RESET;

	  LeftW.PID = 100;

	  HAL_TIM_PWM_Start(&LeftW.htim, LeftW.PWM_Channel);
	  __HAL_TIM_SET_COMPARE(&LeftW.htim, LeftW.PWM_Channel, LeftW.PWM_Value );


	  RightW.hi2c = hi2c1;
	  RightW.htim = htim1;
	  RightW.PWM_Channel = TIM_CHANNEL_4;
	  RightW.PWM_Value = 0;
	  RightW.GPIOx_INA = GPIOA;
	  RightW.GPIOx_INB = GPIOA;
	  RightW.GPIO_Pin_INA = L_INA_Pin;
	  RightW.GPIO_Pin_INB = L_INB_Pin;
	  RightW.PinState_INA   = GPIO_PIN_RESET;
	  RightW.PinState_INB   = GPIO_PIN_RESET;

	  RightW.PID = 100;

	  HAL_TIM_PWM_Start(&RightW.htim, RightW.PWM_Channel);
	  __HAL_TIM_SET_COMPARE(&RightW.htim, RightW.PWM_Channel, RightW.PWM_Value );
}

void SetDir_Speed(float Speed) {

	RightW.Target_speed = Speed;
	LeftW.Target_speed = Speed;

	RightW.PWM_Value = (int32_t) Speed;
	LeftW.PWM_Value = (int32_t) Speed;

	if (LeftW.Target_speed > 0) {
		LeftW.PinState_INA = GPIO_PIN_RESET;
		LeftW.PinState_INB = GPIO_PIN_SET;
		LeftW.Direction = WH_CW;
	};
	if (LeftW.Target_speed < 0) {
		LeftW.PinState_INA = GPIO_PIN_SET;
		LeftW.PinState_INB = GPIO_PIN_RESET;
		LeftW.Direction = WH_CCW;
	};

	if (LeftW.Target_speed == 0) {
		LeftW.PinState_INA = GPIO_PIN_RESET;
		LeftW.PinState_INB = GPIO_PIN_RESET;
		LeftW.Direction = WH_STOP;
	};
	HAL_GPIO_WritePin(LeftW.GPIOx_INA, LeftW.GPIO_Pin_INA, LeftW.PinState_INA);
	HAL_GPIO_WritePin(LeftW.GPIOx_INB, LeftW.GPIO_Pin_INB, LeftW.PinState_INB);

	__HAL_TIM_SET_COMPARE(&LeftW.htim, LeftW.PWM_Channel, LeftW.PWM_Value);

	if (RightW.Target_speed > 0) {
		RightW.PinState_INA = GPIO_PIN_SET;
		RightW.PinState_INB = GPIO_PIN_RESET;
		RightW.Direction = WH_CW;
	};
	if (RightW.Target_speed < 0) {
		RightW.PinState_INA = GPIO_PIN_RESET;
		RightW.PinState_INB = GPIO_PIN_SET;
		RightW.Direction = WH_CCW;
	};

	if (RightW.Target_speed == 0) {
		RightW.PinState_INA = GPIO_PIN_RESET;
		RightW.PinState_INB = GPIO_PIN_RESET;
		RightW.PWM_Value = 0;
		RightW.Direction = WH_STOP;

	};

	HAL_GPIO_WritePin(RightW.GPIOx_INA, RightW.GPIO_Pin_INA,
			RightW.PinState_INA);
	HAL_GPIO_WritePin(RightW.GPIOx_INB, RightW.GPIO_Pin_INB,
			RightW.PinState_INB);

	__HAL_TIM_SET_COMPARE(&RightW.htim, RightW.PWM_Channel, RightW.PWM_Value);

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */


  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  Motor_Init();




  speedr =  0;
  SetDir_Speed(speedr);


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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM11) {
		HAL_IncTick();
		return;
	}

	if (htim->Instance == TIM2) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
			LeftW.angle[i - 1] = LeftW.angle[i];
			RightW.angle[i - 1] = RightW.angle[i];

			LeftW.time_ms_wheel[i - 1] = LeftW.time_ms_wheel[i];
			RightW.time_ms_wheel[i - 1] = RightW.time_ms_wheel[i];

			LeftW.speed[i - 1] = LeftW.speed[i];
			RightW.speed[i - 1] = RightW.speed[i];

		};

		RightW.time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1] = HAL_GetTick();
		LeftW.time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1] = HAL_GetTick();

		LeftW.angle[MAX_ANGLE_WHEEL_ARRAY - 1] = HAL_ReadAS5600_I2Cx(
				LeftW.hi2c);
		RightW.angle[MAX_ANGLE_WHEEL_ARRAY - 1] = HAL_ReadAS5600_I2Cx(
				RightW.hi2c);

		float tmplspeed = -((1000
				* (LeftW.angle[MAX_ANGLE_WHEEL_ARRAY - 1]
						- LeftW.angle[MAX_ANGLE_WHEEL_ARRAY - 2]))
				/ (LeftW.time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1]
						- LeftW.time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 2])) / 360
				* 60;
		float tmprspeed = ((1000
				* (RightW.angle[MAX_ANGLE_WHEEL_ARRAY - 1]
						- RightW.angle[MAX_ANGLE_WHEEL_ARRAY - 2]))
				/ (RightW.time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 1]
						- RightW.time_ms_wheel[MAX_ANGLE_WHEEL_ARRAY - 2]))
				/ 360 * 60;

		if ((tmplspeed * LeftW.speed[MAX_ANGLE_WHEEL_ARRAY - 2]) >= 0) {
			for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
				LeftW.speed[i - 1] = LeftW.speed[i];
			};

			if (fabsf(tmplspeed) < 1) {
				tmplspeed = 0;
			};
			LeftW.speed[MAX_ANGLE_WHEEL_ARRAY - 1] = tmplspeed;
		};

		if ((tmprspeed * RightW.speed[MAX_ANGLE_WHEEL_ARRAY - 2]) >= 0) {
			for (uint8_t i = 1; i < MAX_ANGLE_WHEEL_ARRAY; i++) {
				RightW.speed[i - 1] = RightW.speed[i];
			};
			if (fabsf(tmprspeed) < 1) {
				tmprspeed = 0;
			};
			RightW.speed[MAX_ANGLE_WHEEL_ARRAY - 1] = tmprspeed;
		};

		LeftW.averspeed = 0;
		RightW.averspeed = 0;

		for (uint8_t i = 0; i < MAX_ANGLE_WHEEL_ARRAY - 2; i++) {
			LeftW.averspeed = LeftW.averspeed + LeftW.speed[i];
			RightW.averspeed = RightW.averspeed + RightW.speed[i];
		};

		RightW.averspeed = RightW.averspeed / (MAX_ANGLE_WHEEL_ARRAY - 2);
		LeftW.averspeed = LeftW.averspeed / (MAX_ANGLE_WHEEL_ARRAY - 2);

		if (RightW.averspeed > 7) {
			if (RightW.averspeed - LeftW.averspeed < 0) {
				RightW.PWM_Value = RightW.PWM_Value + 5;
				__HAL_TIM_SET_COMPARE(&RightW.htim, RightW.PWM_Channel,
						RightW.PWM_Value);

				LeftW.PWM_Value = LeftW.PWM_Value - 5;
				__HAL_TIM_SET_COMPARE(&LeftW.htim, LeftW.PWM_Channel,
						LeftW.PWM_Value);

			} else {

				RightW.PWM_Value = RightW.PWM_Value - 5;
				__HAL_TIM_SET_COMPARE(&RightW.htim, RightW.PWM_Channel,
						RightW.PWM_Value);

				LeftW.PWM_Value = LeftW.PWM_Value + 5;
				__HAL_TIM_SET_COMPARE(&LeftW.htim, LeftW.PWM_Channel,
						LeftW.PWM_Value);

			};
		};

	}

}




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
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 1  / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
    {
        // Add code here
        task1msCnt++;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    /* USER CODE END Task10msHandler */
}

/* USER CODE BEGIN Header_Task10msHandler */
/**
* @brief Function implementing the Task10ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task10msHandler */
void Task10msHandler(void *argument)
{
    /* USER CODE BEGIN Task10msHandler */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
    {
        // Add code here

        task10msCnt++;

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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


void Task100msHandler(void *argument)
{
    /* USER CODE BEGIN Task100msHandler */
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for(;;)
    {
        // Add code here
        task100msCnt++;


    	if (newval==1) {
    		newval = 0;
    		 SetDir_Speed(speedr);
    	}

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    /* USER CODE END Task50msHandler */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */


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

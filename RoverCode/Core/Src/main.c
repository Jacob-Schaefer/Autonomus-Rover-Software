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
#include "servocontrol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServoDrivers */
osThreadId_t ServoDriversHandle;
const osThreadAttr_t ServoDrivers_attributes = {
  .name = "ServoDrivers",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Pathfinding */
osThreadId_t PathfindingHandle;
const osThreadAttr_t Pathfinding_attributes = {
  .name = "Pathfinding",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
int turnRadius = 3000; // Straight is anything greater than 2000

uint32_t pMillis;
uint32_t val1 = 0;
uint32_t val2 = 0;

float distanceFront;
float distanceBack;
float distanceRight;
float distanceLeft;
float distanceFrontDown;
float distanceBackDown;
float distanceLeftDown;
float distanceRightDown;


int pulse_durationFront = 0;
int pulse_durationBack = 0;
int pulse_durationLeft = 0;
int pulse_durationRight = 0;
int pulse_durationFrontDown = 0;
int pulse_durationBackDown = 0;
int pulse_durationLeftDown = 0;
int pulse_durationRightDown = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartServoDrivers(void *argument);
void StartPathfinding(void *argument);

/* USER CODE BEGIN PFP */
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int abs(int x)
{
	if(x < 0)
		return -x;
	else
		return x;
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim5,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim5) < us);  // wait for the counter to reach the us input in the parameter
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

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

  /* creation of ServoDrivers */
  ServoDriversHandle = osThreadNew(StartServoDrivers, NULL, &ServoDrivers_attributes);

  /* creation of Pathfinding */
  PathfindingHandle = osThreadNew(StartPathfinding, NULL, &Pathfinding_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Period = 3359;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3359;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 168;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9899;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1999;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 7;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10499;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */
  HAL_TIM_Base_Start(&htim5);
  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15|Ultrasonic1_Trig_Pin|Ultrasonic2_Trig_Pin|Ultrasonic3_Trig_Pin
                          |Ultrasonic4_Trig_Pin|Ultrasonic5_Trig_Pin|Ultrasonic6_Trig_Pin|Ultrasonic7_Trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Ultrasonic7_Echo_Pin PC14 Ultrasonic1_Echo_Pin Ultrasonic2_Echo_Pin
                           Ultrasonic3_Echo_Pin Ultrasonic4_Echo_Pin Ultrasonic5_Echo_Pin Ultrasonic6_Echo_Pin */
  GPIO_InitStruct.Pin = Ultrasonic7_Echo_Pin|GPIO_PIN_14|Ultrasonic1_Echo_Pin|Ultrasonic2_Echo_Pin
                          |Ultrasonic3_Echo_Pin|Ultrasonic4_Echo_Pin|Ultrasonic5_Echo_Pin|Ultrasonic6_Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC15 Ultrasonic1_Trig_Pin Ultrasonic2_Trig_Pin Ultrasonic3_Trig_Pin
                           Ultrasonic4_Trig_Pin Ultrasonic5_Trig_Pin Ultrasonic6_Trig_Pin Ultrasonic7_Trig_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_15|Ultrasonic1_Trig_Pin|Ultrasonic2_Trig_Pin|Ultrasonic3_Trig_Pin
                          |Ultrasonic4_Trig_Pin|Ultrasonic5_Trig_Pin|Ultrasonic6_Trig_Pin|Ultrasonic7_Trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void fireSensors( void )
{
	/**Front**/
	/*
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)) && pMillis +10 > HAL_GetTick());
	val1 = __HAL_TIM_GET_COUNTER(&htim5);

	pMillis = HAL_GetTick();
	while ((HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1)) && pMillis + 50 > HAL_GetTick());
	val2 = __HAL_TIM_GET_COUNTER(&htim5);
	int pulse_durationFront = 0;
	pulse_durationFront = (val2-val1);
	*/
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN1, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN1, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN1)) && pMillis +10 > HAL_GetTick());
	val1 = __HAL_TIM_GET_COUNTER(&htim5);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN1)) && pMillis + 50 > HAL_GetTick());
	val2 = __HAL_TIM_GET_COUNTER(&htim5);

	pulse_durationFront = (val2 - val1);
	osDelay(1);

	/**Back**/
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN2, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN2, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN2)) && pMillis +10 > HAL_GetTick());
	val1 = __HAL_TIM_GET_COUNTER(&htim5);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN2)) && pMillis + 50 > HAL_GetTick());
	val2 = __HAL_TIM_GET_COUNTER(&htim5);

	pulse_durationBack = (val2 - val1);
	osDelay(1);



	/**Left**/
	/*
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN3, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN3, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN3)) && pMillis +10 > HAL_GetTick());
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN3)) && pMillis + 50 > HAL_GetTick());
	uint32_t pulse_durationLeft = __HAL_TIM_GET_COUNTER(&htim5);
	osDelay(1);
	*/

	/**Right**/
	/*
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN4, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN4, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN4)) && pMillis +10 > HAL_GetTick());
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN4)) && pMillis + 50 > HAL_GetTick());
	uint32_t pulse_durationRight = __HAL_TIM_GET_COUNTER(&htim5);
	osDelay(1);
	*/

	/**Front Down**/
	/*
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN5, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN5, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN5)) && pMillis +10 > HAL_GetTick());
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN5)) && pMillis + 50 > HAL_GetTick());
	uint32_t pulse_durationFrontDown = __HAL_TIM_GET_COUNTER(&htim5);
	osDelay(1);
	*/

	/**Back Down**/
	/*
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN6, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN6, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN6)) && pMillis +10 > HAL_GetTick());
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN6)) && pMillis + 50 > HAL_GetTick());
	uint32_t pulse_durationBackDown = __HAL_TIM_GET_COUNTER(&htim5);
	osDelay(1);
	*/

	/**Left Down**/
	/*
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN7, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN7, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN7)) && pMillis +10 > HAL_GetTick());
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN7)) && pMillis + 50 > HAL_GetTick());
	uint32_t pulse_durationLeftDown = __HAL_TIM_GET_COUNTER(&htim5);
	osDelay(1);
	/*

	/**Right Down**/
	/*
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN8, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOC, TRIG_PIN8, GPIO_PIN_RESET);

	pMillis = HAL_GetTick();
	while(!(HAL_GPIO_ReadPin(GPIOC, ECHO_PIN8)) && pMillis +10 > HAL_GetTick());
	__HAL_TIM_SET_COUNTER(&htim5, 0);

	pMillis = HAL_GetTick();
	while((HAL_GPIO_ReadPin(GPIOC, ECHO_PIN8)) && pMillis + 50 > HAL_GetTick());
	uint32_t pulse_durationRightDown = __HAL_TIM_GET_COUNTER(&htim5);
	osDelay(1);
	*/

	calculateDistance(pulse_durationFront,pulse_durationBack,pulse_durationLeft,pulse_durationRight );
	calculateDistanceDown(pulse_durationFrontDown,pulse_durationBackDown,pulse_durationLeftDown,pulse_durationRightDown);

	osDelay(1);

}

void calculateDistance(int pulse_durationFront, int pulse_durationBack, int pulse_durationLeft, int pulse_durationRight)
{

    // Calculate distance based on pulse duration
    // Distance = (pulse duration * speed of sound) / 2
    // Convert distance to inches by Distance * (1/2.54)
    distanceFront = ((pulse_durationFront * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);
    distanceBack = ((pulse_durationBack * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);
    distanceLeft = ((pulse_durationLeft * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);
    distanceRight = ((pulse_durationRight * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);

}

void calculateDistanceDown(int pulse_durationFrontDown,int pulse_durationBackDown,int pulse_durationLeftDown,int pulse_durationRightDown)
{
    distanceFrontDown = ((pulse_durationFrontDown * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);
    distanceBackDown = ((pulse_durationBackDown * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);
    distanceLeftDown = ((pulse_durationLeftDown * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);
    distanceRightDown = ((pulse_durationRightDown * SOUND_SPEED_CM_PER_US) / 2.0) * (1/2.54);
}

void navigateObject()
{
    bool doNotBackUp = (distanceBack < MIN_DISTANCE);
    bool doNotTurnLeft = (distanceLeft < MIN_DISTANCE);
    bool doNotTurnRight = (distanceRight < MIN_DISTANCE);
    bool DropRight = (distanceRightDown > STAN_FLOOR_DISTANCE);
    bool DropLeft = (distanceLeftDown > STAN_FLOOR_DISTANCE);
    bool DropBack = (distanceBackDown > STAN_FLOOR_DISTANCE);


    if (!doNotTurnRight && !DropRight)
    {
        // Turn right
        // Code to turn right...
    	driveServos( 500 );
    	osDelay(500);
        fireSensors();
        if(distanceFront <= MIN_DISTANCE || distanceFrontDown > STAN_FLOOR_DISTANCE)
        {
            navigateObject();

        }
    }
    else if (doNotTurnRight && !doNotTurnLeft || DropRight && !doNotTurnLeft)
    {
        // Turn left
        // Code to turn left...
    	driveServos( -500 );
    	osDelay(500);
    	fireSensors();
        if(distanceFront <= MIN_DISTANCE || distanceFrontDown > STAN_FLOOR_DISTANCE)
        {
            navigateObject();
        }
    }
    else if ((doNotTurnRight || DropRight) && (doNotTurnLeft || DropLeft) && !doNotBackUp)
    {
        // Backup
        // Code to backup...
    	driveServos( 1 );
    	osDelay(500);
    	fireSensors();
        if(distanceFront <= MIN_DISTANCE || distanceFrontDown > STAN_FLOOR_DISTANCE)
        {
            navigateObject();
        }
    }
    else
    {
    	driveServos(-2500 );
    	osDelay(500);
       //osDelay(10000);
    }

}

void standardTravel( void )
{
        fireSensors();
        if (distanceFront <= MIN_DISTANCE)
        {
        	navigateObject(distanceLeft, distanceRight, distanceFront, distanceBack);
		}
        else
        {
        	driveServos(-2500 );
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
 	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartServoDrivers */
/**
* @brief Function implementing the ServoDrivers thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoDrivers */
void StartServoDrivers(void *argument)
{
  /* USER CODE BEGIN StartServoDrivers */
  /* Infinite loop */
  for(;;)
  {
	  /*
	 HAL_ADC_Start(&hadc1);
	 HAL_ADC_PollForConversion(&hadc1, 1);
	 leftrightJoystick = HAL_ADC_GetValue(&hadc1);
	 osDelay(1);

	 HAL_ADC_Start(&hadc1);
	 HAL_ADC_PollForConversion(&hadc1, 1);
	 forwardbackJoystick = HAL_ADC_GetValue(&hadc1);
	 osDelay(1);

	 if( leftrightJoystick < 2000)
	 {
		 leftrightVal = map(leftrightJoystick, 0, 2000, -1, -2000);
	 }
	 else if ( leftrightJoystick > 3000 )
	 {
		 leftrightVal = map(leftrightJoystick, 3000, 4800, -1, 2000);
	 }

	 if (forwardbackJoystick > 3000 ) forwardbackVal = 1;
	 else forwardbackVal = 2;

	  driveServos( leftrightVal, forwardbackVal );
	  */


	  /*
	  for(int i = -2500; i<-500; i+=100)
	  {
		  driveServos( i );
		  osDelay(50);
	  }
	  for(int i = -500; i<0; i+=10)
	  	  {
	  		  driveServos( i );
	  		  osDelay(50);
	  	  }
	  osDelay(1000);
	  for(int i = 0; i<500; i+=10)
	  	  	  {
	  	  		  driveServos( i );
	  	  		  osDelay(50);
	  	  	  }
	  for(int i = 500; i<2500; i+=100)
	  {
		  driveServos( i );
		  osDelay(50);
	  }


	  for(int i = 2500; i>500; i-=100)
	  {
		  driveServos( i );
		  osDelay(50);
	  }
	  for(int i = 500; i>-500; i-=10)
	  	  {
	  		  driveServos( i );
	  		  osDelay(50);
	  	  }
	  for(int i = -500; i>-2500; i-=100)
	  {
		  driveServos( i );
		  osDelay(50);
	  }
	  */

	  //driveServos( -2500, 1 );
	  osDelay(1000);
  }
  /* USER CODE END StartServoDrivers */
}

/* USER CODE BEGIN Header_StartPathfinding */
/**
* @brief Function implementing the Pathfinding thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPathfinding */
void StartPathfinding(void *argument)
{
  /* USER CODE BEGIN StartPathfinding */
  /* Infinite loop */
  for(;;)
  {
	  standardTravel();
	  osDelay(1);
  }
  /* USER CODE END StartPathfinding */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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

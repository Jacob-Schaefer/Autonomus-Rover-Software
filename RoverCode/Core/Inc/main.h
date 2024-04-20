/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int map( int x, int in_min, int in_max, int out_min, int out_max );
int abs(int x);
void delay_us (uint16_t us);

typedef int bool;
	#define FALSE 0
	#define TRUE 1

void calculateDistance(int pulse_durationFront, int pulse_durationBack, int pulse_durationLeft, int pulse_durationRight);
void calculateDistanceDown(int pulse_durationFrontDown,int pulse_durationBackDown,int pulse_durationLeftDown,int pulse_durationRightDown);
void standardTravel();
void navigateObject();
void fireSensors();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ultrasonic7_Echo_Pin GPIO_PIN_13
#define Ultrasonic7_Echo_GPIO_Port GPIOC
#define Ultrasonic1_Trig_Pin GPIO_PIN_0
#define Ultrasonic1_Trig_GPIO_Port GPIOC
#define Ultrasonic1_Echo_Pin GPIO_PIN_1
#define Ultrasonic1_Echo_GPIO_Port GPIOC
#define Ultrasonic2_Trig_Pin GPIO_PIN_2
#define Ultrasonic2_Trig_GPIO_Port GPIOC
#define Ultrasonic2_Echo_Pin GPIO_PIN_3
#define Ultrasonic2_Echo_GPIO_Port GPIOC
#define MotorCh1FWD_Pin GPIO_PIN_0
#define MotorCh1FWD_GPIO_Port GPIOA
#define MotorCh2FWD_Pin GPIO_PIN_1
#define MotorCh2FWD_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ServoCh1_Pin GPIO_PIN_6
#define ServoCh1_GPIO_Port GPIOA
#define ServoCh2_Pin GPIO_PIN_7
#define ServoCh2_GPIO_Port GPIOA
#define Ultrasonic3_Trig_Pin GPIO_PIN_4
#define Ultrasonic3_Trig_GPIO_Port GPIOC
#define Ultrasonic3_Echo_Pin GPIO_PIN_5
#define Ultrasonic3_Echo_GPIO_Port GPIOC
#define ServoCh3_Pin GPIO_PIN_0
#define ServoCh3_GPIO_Port GPIOB
#define ServoCh4_Pin GPIO_PIN_1
#define ServoCh4_GPIO_Port GPIOB
#define MotorCh4FWD_Pin GPIO_PIN_2
#define MotorCh4FWD_GPIO_Port GPIOB
#define MotorCh3FWD_Pin GPIO_PIN_10
#define MotorCh3FWD_GPIO_Port GPIOB
#define Ultrasonic4_Trig_Pin GPIO_PIN_6
#define Ultrasonic4_Trig_GPIO_Port GPIOC
#define Ultrasonic4_Echo_Pin GPIO_PIN_7
#define Ultrasonic4_Echo_GPIO_Port GPIOC
#define Ultrasonic5_Trig_Pin GPIO_PIN_8
#define Ultrasonic5_Trig_GPIO_Port GPIOC
#define Ultrasonic5_Echo_Pin GPIO_PIN_9
#define Ultrasonic5_Echo_GPIO_Port GPIOC
#define MotorCh1BKWD_Pin GPIO_PIN_8
#define MotorCh1BKWD_GPIO_Port GPIOA
#define MotorCh2BKWD_Pin GPIO_PIN_9
#define MotorCh2BKWD_GPIO_Port GPIOA
#define MotorCh3BKWD_Pin GPIO_PIN_10
#define MotorCh3BKWD_GPIO_Port GPIOA
#define MotorCh4BKWD_Pin GPIO_PIN_11
#define MotorCh4BKWD_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Ultrasonic6_Trig_Pin GPIO_PIN_10
#define Ultrasonic6_Trig_GPIO_Port GPIOC
#define Ultrasonic6_Echo_Pin GPIO_PIN_11
#define Ultrasonic6_Echo_GPIO_Port GPIOC
#define Ultrasonic7_Trig_Pin GPIO_PIN_12
#define Ultrasonic7_Trig_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SOUND_SPEED_CM_PER_US 0.0343
#define MIN_DISTANCE 12
#define STAN_FLOOR_DISTANCE 12

#define TRIG_PIN1 GPIO_PIN_0
#define ECHO_PIN1 GPIO_PIN_1
#define TRIG_PIN2 GPIO_PIN_2
#define ECHO_PIN2 GPIO_PIN_3
#define TRIG_PIN3 GPIO_PIN_4
#define ECHO_PIN3 GPIO_PIN_5
#define TRIG_PIN4 GPIO_PIN_6
#define ECHO_PIN4 GPIO_PIN_7

#define TRIG_PIN5 GPIO_PIN_8
#define ECHO_PIN5 GPIO_PIN_9
#define TRIG_PIN6 GPIO_PIN_10
#define ECHO_PIN6 GPIO_PIN_11
#define TRIG_PIN7 GPIO_PIN_12
#define ECHO_PIN7 GPIO_PIN_13
#define TRIG_PIN8 GPIO_PIN_14
#define ECHO_PIN8 GPIO_PIN_15
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

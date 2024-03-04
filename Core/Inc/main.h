/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "can_config.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// Debug defines
#define NO_CALIBRATION
#define NO_WHEEL_AQ
//#define NO_APPS_AQ
//#define NO_ERRORS_CHECKS
//#define NO_BPPS_AQ
#define NO_PRES_AQ
#define NO_ST_WHEEL_AQ

//#define NO_APP_ERRORS
//#define NO_BPPS_ERRORS
#define NO_ST_WHEEL_ERRORS
#define NO_PRESSURE_ERRORS

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define NUMBER_OF_ERRORS 7U
#define NUMBER_OF_USED_ADC_CHANNELS 7U

typedef enum
{
	CALIBRATION_START,
	BRAKE_MIN_CALIBRATION_START,
	BRAKE_MIN,
	BRAKE_MAX_CALIBRATION_START,
	BRAKE_MAX,
	ACCELERATOR_MIN_CALIBRATION_START,
	ACCELERATOR_MIN,
	ACCELERATOR_MAX_CALIBRATION_START,
	ACCELERATOR_MAX,
} calibrationStages;

typedef enum
{
	MOST_SIGNIFICANT_BYTE = 0U,
	LESS_SIGNIFICANT_BYTE = 1U,
	SHIFT_BYTE = 8U,
} dataManipulators;

typedef enum
{
	SHORT_TO_GROUND,
	SHORT_TO_VCC,
	SHORT_BETWEEN_SIGNAL_LINES,
	OUT_OF_RANGE,
	DEVIATION_BETWEEN_SIGNALS,
	UNDER_PRESSURE,
	OVER_PRESSURE,
	NO_ERRORS,
} errorCode;

typedef struct signal
{
	uint32_t *raw_value_ptr;
	uint32_t value;
	uint32_t max;
	uint32_t min;
	uint32_t physicallMax;
	uint32_t physicallMin;
	uint32_t range;
	uint32_t channel;
	uint32_t minStage;
	uint32_t maxStage;
	uint8_t *position;
	uint32_t errorsWatchDog;
	void (*errorHandler[NUMBER_OF_ERRORS])();
} Signal;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void calibrate(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define APPS_SIGNAL_A_uC_Pin GPIO_PIN_1
#define APPS_SIGNAL_A_uC_GPIO_Port GPIOA
#define APPS_SIGNAL_B_uC_Pin GPIO_PIN_2
#define APPS_SIGNAL_B_uC_GPIO_Port GPIOA
#define BPPS_SIGNAL_1_uC_Pin GPIO_PIN_4
#define BPPS_SIGNAL_1_uC_GPIO_Port GPIOA
#define BPPS_SIGNAL_2_uC_Pin GPIO_PIN_5
#define BPPS_SIGNAL_2_uC_GPIO_Port GPIOA
#define PRES_SENS_1_uC_Pin GPIO_PIN_6
#define PRES_SENS_1_uC_GPIO_Port GPIOA
#define PRES_SENS_2_uC_Pin GPIO_PIN_7
#define PRES_SENS_2_uC_GPIO_Port GPIOA
#define ST_WHEEL_SENS_Pin GPIO_PIN_0
#define ST_WHEEL_SENS_GPIO_Port GPIOB
#define PCB_LED_RED_Pin GPIO_PIN_1
#define PCB_LED_RED_GPIO_Port GPIOB
#define PDB_LED_GREEN_Pin GPIO_PIN_2
#define PDB_LED_GREEN_GPIO_Port GPIOB
#define R_WHEEL_SENS_B_uC_Pin GPIO_PIN_15
#define R_WHEEL_SENS_B_uC_GPIO_Port GPIOA
#define R_WHEEL_SENS_A_uC_Pin GPIO_PIN_3
#define R_WHEEL_SENS_A_uC_GPIO_Port GPIOB
#define L_WHEEL_SENS_A_uC_Pin GPIO_PIN_4
#define L_WHEEL_SENS_A_uC_GPIO_Port GPIOB
#define L_WHEEL_SENS_B_uC_Pin GPIO_PIN_5
#define L_WHEEL_SENS_B_uC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

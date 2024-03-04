/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_config.h"
#include "errors.h"
#include "frontSensInit.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
static const uint8_t numberOfEncoderDiscTeeth = 1;
static const uint8_t WheelSpeedMeasureTime_ms = 100;
static const uint16_t maxWheelSpeed = 1000;
static const uint8_t maxImplausibilityTime_ms = 80;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Calibration variables
bool Write_isInCalibrationState = false;
uint8_t calibrationStage = CALIBRATION_START;
uint8_t Read_CurrentCalibratedSensorValue[2];

// ADC buffer
uint32_t ADC_buffer[NUMBER_OF_USED_ADC_CHANNELS];

// BPPS variables
uint8_t Read_BPPSPosition[2] = {0,0};
Signal signalA_BPPS, signalB_BPPS;

// APPS variables
uint8_t Read_APPSPosition[2] = {0,0};
Signal signalA_APPS, signalB_APPS;

// ST_WHEEL variables
uint8_t Read_ST_WHEELAngle[2] = {0,0};
Signal signal_ST_WHEEL;

// R/L_WHHEL variables
uint32_t R_WHEELStartTime, L_WHEELStartTime;
uint8_t Read_R_WHEELSpeed[2] = {0,0};
uint8_t Read_L_WHEELSpeed[2] = {0,0};
bool R_WHEELReadDone, L_WHEELReadDone;

// Pressure sensors variables
uint8_t Read_PressureOneValue[2] = {0,0};
uint8_t Read_PressureTwoValue[2] = {0,0};
Signal signal_PressureOne, signal_PressureTwo;

// Error variable
bool anyErrorOccurred;

void beginCalibrationState(bool *isInCalibrationState);
void calibrate();

void updatePedalPositionSensor(Signal *signalA, Signal *signalB);
void updateST_WHEEL(Signal *signal);
void updateWHEELS();
void updatePressureValue(Signal *signal);

void fixValuesToSetRanges(Signal *signalA, Signal *signalB);
void calculatePosition(Signal *signalA, Signal *signalB);
uint16_t average(uint32_t signalA_value, uint32_t signalB_value);
void calculateAngle(Signal *signal);
void calculateSpeedAndCheckIfInRange(uint32_t starTime, uint8_t wheelSpeed[], void (*outOfRange)());
void calculatePressure(Signal *signal);

void HAL_TIM_IC_CaputreCallback(TIM_HandleTypeDef *htim);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  CAN_Init();
  HAL_CAN_Start(&hcan);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  frontSensInit();
  HAL_ADC_Start_DMA(&hadc, ADC_buffer, NUMBER_OF_USED_ADC_CHANNELS);

  // Start the timer for error handling
  #ifndef NO_ERRORS_CHECKS
  HAL_TIM_Base_Start_IT(&htim16);
  #endif

  #ifndef NO_CALIBRATION
  	  beginCalibrationState(&Write_isInCalibrationState);
  #endif

	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		#ifndef NO_APPS_AQ
		updatePedalPositionSensor(&signalA_APPS, &signalB_APPS);
		#endif
		#ifndef NO_BPPS_AQ
		updatePedalPositionSensor(&signalA_BPPS, &signalB_BPPS);
		#endif
		#ifndef NO_ST_WHEEL_AQ
		updateST_WHEEL(&signal_ST_WHEEL);
		#endif
		#ifndef NO_WHEEL_AQ
		updateWHEELS();
		#endif
		#ifndef NO_PRES_AQ
		updatePressureValue(&signal_PressureOne);
		updatePressureValue(&signal_PressureTwo);
		#endif
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/** beginCalibrationState
 * @brief Function preventing main program to run before the calibration process is completed.
 *
 * @param bool *isInCalibrationState pointer to a boolean defining if the board is in calibration state
 *
 * @retval None
 **/
void beginCalibrationState(bool *isInCalibrationState)
{
	HAL_GPIO_WritePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin, GPIO_PIN_SET);

	while (*isInCalibrationState)
	{
		// TEMPORARY: 27.07.2023
		updatePedalPositionSensor(&signalA_BPPS, &signalB_BPPS);
		updatePedalPositionSensor(&signalA_APPS, &signalB_APPS);
	}

	

	HAL_GPIO_WritePin(PCB_LED_RED_GPIO_Port, PCB_LED_RED_Pin, GPIO_PIN_RESET);
}

/** calibrate
 * @brief Function that handles the calibration process.
 *		  1. First function call is an entrance to the calibration screen.
 *		  2. Second function call is a start of calibration of the first value.
 *		  3. Third function call is the end of the first calibration stage and beginning of the second one.
 *		  The process for every other value that has to be calibrated looks the same as points 2 -> 3.
 *		  At the end, calibratioStage is set to CALIBRAIONT_START in case a driver would like to repeat the process.
 *
 * @retval None
 **/
void calibrate()
{
	switch (calibrationStage)
	{
	case CALIBRATION_START:
		Read_CurrentCalibratedSensorValue[MOST_SIGNIFICANT_BYTE] = 1;
		Read_CurrentCalibratedSensorValue[LESS_SIGNIFICANT_BYTE] = 1;
		break;
	case BRAKE_MIN_CALIBRATION_START:
		break;
	case BRAKE_MIN:
		updatePedalPositionSensor(&signalA_BPPS, &signalB_BPPS);
		break;
	case BRAKE_MAX_CALIBRATION_START:
		break;
	case BRAKE_MAX:
		updatePedalPositionSensor(&signalA_BPPS, &signalB_BPPS);
		break;
	case ACCELERATOR_MIN_CALIBRATION_START:
		break;
	case ACCELERATOR_MIN:
		updatePedalPositionSensor(&signalA_APPS, &signalB_APPS);
		break;
	case ACCELERATOR_MAX_CALIBRATION_START:
		break;
	case ACCELERATOR_MAX:
		updatePedalPositionSensor(&signalA_APPS, &signalB_APPS);
		calibrationStage = CALIBRATION_START;
		break;
	default:
		Error_Handler();
		break;
	}

	calibrationStage++;
}

/** updatePedalPositionSensor
 * @brief Function reads values of the pedal position sensor from ADC, checks them for errors and calculates pedal's current position.
 *
 * @param Signal *signalA pointer to the Signal structure
 * @param Signal *signalB pointer to the Signal structure
 *
 * @retval None
 **/
void updatePedalPositionSensor(Signal *signalA, Signal *signalB)
{
	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_SET);

	/*
	 * Signal value pointed to by the 'raw_value_ptr' might be in a constant change due to the ADC DMA operating on it,
	 * so current (in terms of time when it was saved to the corresponding 'value' element of the 'Signal' structure)
	 * values are used to further calculations.
	 */
	signalA->value = *(signalA->raw_value_ptr);
	signalB->value = *(signalB->raw_value_ptr);

	if (calibrationStage == signalA->minStage)
	{
		signalA->min = signalA->value;
		signalB->min = signalB->value;
	}
	else if (calibrationStage == signalA->maxStage)
	{
		signalA->max = signalA->value;
		signalB->max = signalB->value;
		signalA->range = signalA->max - signalA->min;
		signalB->range = signalB->max - signalB->min;
	}

    #ifdef NO_CALIBRATION
	signalA->range = signalA->max - signalA->min;
	signalB->range = signalB->max - signalB->min;
	#endif

	if (!Write_isInCalibrationState || (signalA->maxStage < calibrationStage))
	{
		fixValuesToSetRanges(signalA, signalB);
	}

	calculatePosition(signalA, signalB);

	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_RESET);
}

/** updateST_WHEEL
 * @brief Function reads value of the steering wheel position sensor from ADC, checks it for errors and calculates steering wheel's current angle.
 *
 * @retval None
 **/
void updateST_WHEEL(Signal *signal)
{
	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_SET);

	signal->value = *(signal->raw_value_ptr);

	calculateAngle(signal);

	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_RESET);
}

/** updatePressureValue
 * @brief Function reads value of the pressure sensor from ADC, checks it for errors and calculates pressure in [bars].
 *
 * @retval None
 **/
void updatePressureValue(Signal *signal)
{
	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_SET);

	signal->value = *(signal->raw_value_ptr);

	calculatePressure(signal);

	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_RESET);
}

/** updateWHEELS
 * @brief Function updates values of current speeds of both front wheels.
 *
 * @retval None
 **/
void updateWHEELS()
{
	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_SET);

	Read_R_WHEELSpeed[MOST_SIGNIFICANT_BYTE] = 0;
	Read_R_WHEELSpeed[LESS_SIGNIFICANT_BYTE] = 0;
	Read_L_WHEELSpeed[MOST_SIGNIFICANT_BYTE] = 0;
	Read_L_WHEELSpeed[LESS_SIGNIFICANT_BYTE] = 0;

	R_WHEELReadDone = false;
	L_WHEELReadDone = false;
	uint32_t timer = HAL_GetTick();
	// L_WHEEL TIM3
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	L_WHEELStartTime = HAL_GetTick();
	// R_WHEEL TIM2
	HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
	R_WHEELStartTime = HAL_GetTick();

	// In case the measure time is higher than a time needed to achieve speed of 1 [RPM], stop the process and assume speed is zero.
	while (!R_WHEELReadDone || !L_WHEELReadDone)
	{
		if ((HAL_GetTick() - timer) > WheelSpeedMeasureTime_ms)
		{
			HAL_TIM_Encoder_Stop_IT(&htim3, TIM_CHANNEL_ALL);
			HAL_TIM_Encoder_Stop_IT(&htim2, TIM_CHANNEL_ALL);

			R_WHEELReadDone = true;
			L_WHEELReadDone = true;
		}
	}

	// TODO error checks

	HAL_GPIO_WritePin(PDB_LED_GREEN_GPIO_Port, PDB_LED_GREEN_Pin, GPIO_PIN_RESET);
}

/** HAL_TIM_IC_CaputreCallback
 * @brief Function handling interrupts from two timers. It is used to calculate the speed of front wheels.
 *
 * @retval None
 **/
void HAL_TIM_IC_CaputreCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		// L_WHEEL
		HAL_TIM_Encoder_Stop_IT(&htim3, TIM_CHANNEL_ALL);
		calculateSpeedAndCheckIfInRange(L_WHEELStartTime, Read_L_WHEELSpeed, LeftWheelSensorOutOfRangeError_Handler);

		L_WHEELReadDone = true;
	}
	else if (htim == &htim2)
	{
		// R_WHEEL
		HAL_TIM_Encoder_Stop_IT(&htim2, TIM_CHANNEL_ALL);
		calculateSpeedAndCheckIfInRange(R_WHEELStartTime, Read_R_WHEELSpeed, RightWheelSensorOutOfRangeError_Handler);

		R_WHEELReadDone = true;
	}
}

/** HAL_TIM_PeriodElapsedCallback
 * @brief Function handling interrupt call for error checks.
 *
 * @retval None
 **/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim16 )
  {
    uint32_t now;

	errorCode error;

	// Check APPS for errors
	#ifndef NO_APP_ERRORS
	now = HAL_GetTick();
	error = checkPositionSensorForErrors(&signalA_APPS, &signalB_APPS);
	if(error == NO_ERRORS)
	{
		signalA_APPS.errorsWatchDog = now;
	}else if((now - signalA_APPS.errorsWatchDog) >= maxImplausibilityTime_ms)
	{
		signalA_APPS.errorHandler[error]();
	}
	#endif

	// Check BPPS for errors
	#ifndef NO_BPPS_ERRORS
	now = HAL_GetTick();
	error = checkPositionSensorForErrors(&signalA_BPPS, &signalB_BPPS);
	if(error == NO_ERRORS)
	{
		signalA_BPPS.errorsWatchDog = now;
	}else if((now - signalA_BPPS.errorsWatchDog) >= maxImplausibilityTime_ms)
	{
		signalA_BPPS.errorHandler[error]();
	}
	#endif

	// Check ST_WHEEL for errors
	#ifndef NO_ST_WHEEL_ERRORS
	now = HAL_GetTick();
	error = checkST_WHEELforErrors(*(signal_ST_WHEEL.raw_value_ptr));
	if(error == NO_ERRORS)
	{
		signal_ST_WHEEL.errorsWatchDog = now;
	}else if((now - signal_ST_WHEEL.errorsWatchDog) >= maxImplausibilityTime_ms)
	{
		signal_ST_WHEEL.errorHandler[error]();
	}
	#endif

	// Check pressure sensors for errors
	#ifndef NO_PRESSURE_ERRORS
	now = HAL_GetTick();
	error = checkPressureSenorForErrors(&signal_PressureOne);
	if(error == NO_ERRORS)
	{
		signal_PressureOne.errorsWatchDog = now;
	}else if((now - signal_PressureOne.errorsWatchDog) >= maxImplausibilityTime_ms)
	{
		signal_PressureOne.errorHandler[error]();
	}

	now = HAL_GetTick();
	error = checkPressureSenorForErrors(&signal_PressureTwo);
	if(error == NO_ERRORS)
	{
		signal_PressureTwo.errorsWatchDog = now;
	}else if((now - signal_PressureTwo.errorsWatchDog) >= maxImplausibilityTime_ms)
	{
		signal_PressureTwo.errorHandler[error]();
	}
	#endif
  }
}

/** calculatePosition
 * @brief Function calculates position of a pedal position sensor.
 *
 * @param Signal *signalA pointer to the Signal structure
 * @param Signal *signalB pointer to the Signal structure
 *
 * @retval None
 **/
void calculatePosition(Signal *signalA, Signal *signalB)
{
	// Calculate position or if in calibration state, update the value
	if (!Write_isInCalibrationState || (signalA->maxStage < calibrationStage))
	{
		uint32_t signalA_fixed = signalA->value - signalA->min;
		uint32_t signalB_fixed = signalB->value - signalB->min;

		uint16_t value = average(signalA_fixed, signalB_fixed);
		uint16_t range = average(signalA->range, signalB->range);
		uint16_t position = ((float)value / (float)range) * 1000; // [%%] - per mille

		/*
		 * If position is lower than 50%% -> 5%, set it to 0.
		 */
		position = position < 50 ? 0 : position;

		signalA->position[MOST_SIGNIFICANT_BYTE] = (position >> SHIFT_BYTE);
		signalA->position[LESS_SIGNIFICANT_BYTE] = position;
	}
	else
	{
		uint16_t avg_value = average(signalA->value, signalB->value);
		Read_CurrentCalibratedSensorValue[MOST_SIGNIFICANT_BYTE] = (avg_value >> SHIFT_BYTE);
		Read_CurrentCalibratedSensorValue[LESS_SIGNIFICANT_BYTE] = avg_value;
	}
}

/** calculatePressure
 * @brief Function calculates pressure based on the ADC reading and pre-calculated formula.
 *
 * @param Signal *signal pointer to the Signal structure
 *
 * @retval None
 **/
void calculatePressure(Signal *signal)
{
	/*
	 * According to data sheet pressure sensor has a linear characteristic.
	 *
	 * 373 is a value corresponding to 0.5V output of the pressure sensors, which means - according to the data sheet - 0 [bar] pressure.
	 * 3351 is a value corresponding to 4.5V output of the pressure sensors, which means - according to the data sheet - 250 [bar] pressure.
	 *
	 * For y = 0 [bar], x = 373 [ADC measure reading], so
	 * 0 = 373 * a + b
	 * For y = 260 [bar], x = 3351 [ADC measure reading], so
	 * 260 = 3351 * a + b
	 * So
	 * a = 0.0873069
	 * and
	 * b = 32.5654737
	 */

	static const float coefficientA = 0.0873069;
	static const float coefficientB = 32.5654737;

	uint16_t pressure = (uint16_t)(signal->value * coefficientA - coefficientB);
	

	signal->position[MOST_SIGNIFICANT_BYTE] = (pressure >> SHIFT_BYTE);
	signal->position[LESS_SIGNIFICANT_BYTE] = pressure;
}

/** average
 * @brief Function calculates average value of two given parameters.
 *
 * @param uint32_t signalA_value first value
 * @param uint32_t signalB_value second value
 *
 * @retval uint16_t average value of two given parameters.
 **/
uint16_t average(uint32_t first_value, uint32_t second_value)
{
	return (uint16_t)((first_value + second_value) / 2);
}

/** calculateAngle
 * @brief Function calculates angle of a steering wheel sensor.
 *
 * @param Signal signal
 *
 * @retval None
 **/
void calculateAngle(Signal *signal)
{
	/*
	 * According to data sheet steering wheel sensor has a linear characteristic.
	 * So the following formula is a linear function y = a * x + b. Where 'x' is an angle in degrees and 'y' is a voltage.
	 * For x = 0 [deg], y = ST_WHEEL_MIN
	 * so: b = ST_WHEEL_MIN
	 * For x = 360 [deg], y = ST_WHEEL_MAX
	 * so: a = (ST_WHEEL_MAX - ST_WHEEL_MIN) / 360
	 * Finally: y = (ST_WHEEL_MAX - ST_WHEEL_MIN) / 360 * x + ST_WHEEL_MIN
	 * So: x = 360 * (y - ST_WHEEL_MIN) / (ST_WHEEL_MAX - ST_WHEEL_MIN)
	 */
	uint16_t angle = (float)(360 * (signal->value - signal->min)) / (float)(signal->max - signal->min);
	Read_ST_WHEELAngle[MOST_SIGNIFICANT_BYTE] = (angle >> SHIFT_BYTE);
	Read_ST_WHEELAngle[LESS_SIGNIFICANT_BYTE] = angle;
}

/** calculateSpeedAndCheckIfInRange
 * @brief Function calculates speed of a wheel.
 *
 * @param uint32_t starTime time when speed acquisition has stared.
 * 		  uint8_t wheelSpeed[] pointer to an array that holds current speed of a wheel and is used as an CAN response.
 *		  void (*outOfRange)() pointer to a function that throws an error of speed being out of physical range
 *
 * @retval None
 **/
void calculateSpeedAndCheckIfInRange(uint32_t starTime, uint8_t wheelSpeed[], void (*outOfRange)())
{
	uint32_t now = HAL_GetTick();

	/*
	 * 'time' variable holds the time( in seconds) between acquisition start and the moment when encoder spotted a teeth.
	 * '1.0 / time' is a frequency and it is multiplied by 60( seconds) and divided by the number of
	 * teeth on the encoder disc to get the rotations per minute.
	 */
	double time = ((double)now - starTime) / 1000;
	uint16_t speed = 1.0 / time * 60.0 / numberOfEncoderDiscTeeth;

	if (speed > maxWheelSpeed)
	{
		outOfRange();
	}

	wheelSpeed[MOST_SIGNIFICANT_BYTE] = (speed >> SHIFT_BYTE);
	wheelSpeed[LESS_SIGNIFICANT_BYTE] = speed;
}

/** fixValuesToSetRanges
 * @brief Function fixes signals values to their ranges.
 *
 * @param Signal *signalA pointer to the Signal structure
 * 		  Signal *signalB pointer to the Signal structure
 *
 * @retval None
 **/
void fixValuesToSetRanges(Signal *signalA, Signal *signalB)
{
	signalA->value = signalA->value> signalA->max ? signalA->max : signalA->value;
	signalA->value = signalA->value < signalA->min ? signalA->min : signalA->value;
	signalB->value = signalB->value > signalB->max ? signalB->max : signalB->value;
	signalB->value = signalB->value < signalB->min ? signalB->min : signalB->value;
}
/* USER CODE END 4 */

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

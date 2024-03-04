#include "frontSensInit.h"
#include "errors.h"

/* NOTE:
* Following constants have to be checked experimentally
* According to data sheet these two should be 90% VCC and 10% VCC
*/ 

static const uint16_t stWheelMaxValue = 4095;
static const uint16_t stWheelMinValue = 0;

static const uint16_t appsSingalAPhysicalMaxValue = 2850;
static const uint16_t appsSignalAPhysicalMinValue = 1780;
static const uint16_t appsSignalBPhysicalMaxValue = 2850;
static const uint16_t appsSignalBPhysicalMinValue = 1500;

static const uint16_t bppsSignalAPhysicalMaxValue = 4095;
static const uint16_t bppsSignalAPhysicalMinValue = 0;
static const uint16_t bppsSignalBPhysicalMaxValue = 4095;
static const uint16_t bppsSignalBPhysicalMinValue = 0;

static const uint16_t pressSensorMaxValue = 4096;
static const uint16_t pressSensorMinValue = 0;

typedef enum {
	APPS_SIGNAL_A_POSITION,
	APPS_SIGNAL_B_POSITION,
	BPPS_SIGNAL_1_POSITION,
	BPPS_SIGNAL_2_POSITION,
	PRES_SENS_1_POSITION,
	PRES_SENS_2_POSITION,
	ST_WHEEL_SENS_POSITION
} valuePositionInADCbuffer;

/* NOTE:
* Following externs are defined in main.c
*/
extern uint32_t ADC_buffer[NUMBER_OF_USED_ADC_CHANNELS];

// Calibration variable
extern bool Write_isInCalibrationState;

// Brake pedal position sensor variables
extern uint8_t Read_BPPSPosition[2];
extern Signal signalA_BPPS, signalB_BPPS;

// Accelerator pedal position sensor variables
extern uint8_t Read_APPSPosition[2];
extern Signal signalA_APPS, signalB_APPS;

// Steering wheel variable
extern Signal signal_ST_WHEEL;

// Pressure sensors variables
extern uint8_t Read_PressureOneValue[2];
extern uint8_t Read_PressureTwoValue[2];
extern Signal signal_PressureOne, signal_PressureTwo;

/** frontSensInit
 * @brief Function to set basic parameters of signal structures such as their physical boundaries, error handlers, ADC channels etc.
 *
 * @retval None.
 **/
void frontSensInit(void)
{
	// Calibration
	#ifndef NO_CALIBRATION
		Write_isInCalibrationState = true;
  	#endif

	uint32_t now = HAL_GetTick();


	// Steering wheel
	signal_ST_WHEEL.max = stWheelMaxValue;
	signal_ST_WHEEL.min = stWheelMinValue;
	signal_ST_WHEEL.errorHandler[SHORT_TO_GROUND] = SteeringWheelSensorShortCircuitToGroundError_Handler;
	signal_ST_WHEEL.errorHandler[SHORT_TO_VCC] = SteeringWheelSensorShortCircuitToVccError_Handler;
	signal_ST_WHEEL.channel = ADC_CHANNEL_8;
	signal_ST_WHEEL.raw_value_ptr = ADC_buffer + ST_WHEEL_SENS_POSITION;
	signal_ST_WHEEL.errorsWatchDog = now;

	// Accelerator pedal position sensor
	signalA_APPS.channel = ADC_CHANNEL_1;
	signalB_APPS.channel = ADC_CHANNEL_2;

	signalA_APPS.errorHandler[DEVIATION_BETWEEN_SIGNALS] = AcceleratorSensorDeviationBetweenTwoSignalsError_Handler;
	signalA_APPS.errorHandler[OUT_OF_RANGE] = AcceleratorSensorOutOfRangeError_Handler;
	signalA_APPS.errorHandler[SHORT_BETWEEN_SIGNAL_LINES] = AcceleratorSensorShortCircuitBetweenSignalLinesError_Handler;
	signalA_APPS.errorHandler[SHORT_TO_GROUND] = AcceleratorSensorShortCircuitToGroundError_Handler;
	signalA_APPS.errorHandler[SHORT_TO_VCC] = AcceleratorSensorShortCircuitToVccError_Handler;

	signalA_APPS.minStage = ACCELERATOR_MIN;
	signalA_APPS.maxStage = ACCELERATOR_MAX;

	signalA_APPS.position = Read_APPSPosition;

	signalA_APPS.physicallMax = appsSingalAPhysicalMaxValue;
	signalA_APPS.physicallMin = appsSignalAPhysicalMinValue;
	signalB_APPS.physicallMax = appsSignalBPhysicalMaxValue;
	signalB_APPS.physicallMin = appsSignalBPhysicalMinValue;

	signalA_APPS.max = signalA_APPS.physicallMax;
	signalA_APPS.min = signalA_APPS.physicallMin;
	signalB_APPS.max = signalB_APPS.physicallMax;
	signalB_APPS.min = signalB_APPS.physicallMin;

	signalA_APPS.raw_value_ptr = ADC_buffer + APPS_SIGNAL_A_POSITION;
	signalB_APPS.raw_value_ptr = ADC_buffer + APPS_SIGNAL_B_POSITION;
	signalA_APPS.errorsWatchDog = now;

	// Brake pedal position sensor
	signalA_BPPS.channel = ADC_CHANNEL_4;
	signalB_BPPS.channel = ADC_CHANNEL_5;

	signalA_BPPS.errorHandler[DEVIATION_BETWEEN_SIGNALS] = BrakeSensorDeviationBetweenTwoSignalsError_Handler;
	signalA_BPPS.errorHandler[OUT_OF_RANGE] = BrakeSensorOutOfRangeError_Handler;
	signalA_BPPS.errorHandler[SHORT_BETWEEN_SIGNAL_LINES] = BrakeSensorShortCircuitBetweenSignalLinesError_Handler;
	signalA_BPPS.errorHandler[SHORT_TO_GROUND] = BrakeSensorShortCircuitToGroundError_Handler;
	signalA_BPPS.errorHandler[SHORT_TO_VCC] = BrakeSensorShortCircuitToVccError_Handler;

	signalA_BPPS.minStage = BRAKE_MIN;
	signalA_BPPS.maxStage = BRAKE_MAX;

	signalA_BPPS.position = Read_BPPSPosition;

	signalA_BPPS.physicallMax = bppsSignalAPhysicalMaxValue;
	signalA_BPPS.physicallMin = bppsSignalAPhysicalMinValue;
	signalB_BPPS.physicallMax = bppsSignalBPhysicalMaxValue;
	signalB_BPPS.physicallMin = bppsSignalBPhysicalMinValue;

	signalA_BPPS.max = signalA_BPPS.physicallMax;
	signalA_BPPS.min = signalA_BPPS.physicallMin;
	signalB_BPPS.max = signalB_BPPS.physicallMax;
	signalB_BPPS.min = signalB_BPPS.physicallMin;

	signalA_BPPS.raw_value_ptr = ADC_buffer + BPPS_SIGNAL_1_POSITION;
	signalB_BPPS.raw_value_ptr = ADC_buffer + BPPS_SIGNAL_2_POSITION;
	signalA_BPPS.errorsWatchDog = now;

	// Pressure sensors
	signal_PressureOne.channel = ADC_CHANNEL_6;
	signal_PressureTwo.channel = ADC_CHANNEL_7;

	signal_PressureOne.max = pressSensorMaxValue;
	signal_PressureOne.min = pressSensorMinValue;
	signal_PressureTwo.max = pressSensorMaxValue;
	signal_PressureTwo.min = pressSensorMinValue;

	signal_PressureOne.position = Read_PressureOneValue;
	signal_PressureTwo.position = Read_PressureTwoValue;

	signal_PressureOne.errorHandler[UNDER_PRESSURE] = PressureSensorUnderPressureError_Handler;
	signal_PressureOne.errorHandler[OVER_PRESSURE] = PressureSensorOverPressureError_Handler;
	signal_PressureTwo.errorHandler[UNDER_PRESSURE] = PressureSensorUnderPressureError_Handler;
	signal_PressureTwo.errorHandler[OVER_PRESSURE] = PressureSensorOverPressureError_Handler;

	signal_PressureOne.raw_value_ptr = ADC_buffer + PRES_SENS_1_POSITION;
	signal_PressureTwo.raw_value_ptr = ADC_buffer + PRES_SENS_2_POSITION;
	signal_PressureOne.errorsWatchDog = now;
	signal_PressureTwo.errorsWatchDog = now;
}

#include "errors.h"

extern bool anyErrorOccurred;

/** checkPositionSensorForErrors
 * @brief Function checks position sensor for errors such as: short circuit to the ground, short circuit to the VCC,
 * 		  short circuit between the signal lines, out of range values and deviation of more than ten percentage points between two sensor values.
 * 
 * @param Signal *signalA - pointer to the Signal structure.
 * @param Signal *signalB - pointer to the Signal structure.
 *
 * @retval if any errors occurred - return error id, if none - return NO_ERRORS.
 **/
errorCode checkPositionSensorForErrors(Signal *signalA, Signal *signalB)
{
	errorCode error = NO_ERRORS;

	uint32_t valueA = *(signalA->raw_value_ptr);
	uint32_t valueB = *(signalB->raw_value_ptr);

	if ((valueA <= (GROUND_VOLTAGE + ERROR_MARGIN)) || (valueB <= (GROUND_VOLTAGE + ERROR_MARGIN)))
	{
		error = SHORT_TO_GROUND;
	}

	if ((valueA >= (VCC_VOLTAGE - ERROR_MARGIN)) || (valueB >= (VCC_VOLTAGE - ERROR_MARGIN)))
	{
		error = SHORT_TO_VCC;
	}

	bool shortBetweenSignalLines = abs(valueA - valueB) < ERROR_MARGIN;

	if (shortBetweenSignalLines)
	{
		error = SHORT_BETWEEN_SIGNAL_LINES;
	}

	double positionA = (double)(valueA - signalA->min) / (signalA->max - signalA->min) * 100;
	double positionB = (double)(valueB - signalB->min) / (signalB->max - signalB->min) * 100;
	if (fabs(positionA - positionB) >= 10.0)
	{
		error = DEVIATION_BETWEEN_SIGNALS;
	}

	bool signalA_belowRange = valueA < signalA->physicallMin;
	bool signalA_overRange = valueA > signalA->physicallMax;
	bool signalB_belowRange = valueB < signalB->physicallMin;
	bool signalB_overRange = valueB > signalB->physicallMax;

	if (signalA_belowRange || signalA_overRange || signalB_belowRange || signalB_overRange)
	{
		error = OUT_OF_RANGE;
	}

	return error;
}

/** checkST_WHEELforErrors
 * @brief Function checks steering wheel sensor for errors such as: short circuit to the ground and short circuit to the VCC.
 * 
 * @param uint32_t voltage - raw value read from ADC.
 *
 * @retval if any errors occurred - return error id, if none - return NO_ERRORS.
 **/
errorCode checkST_WHEELforErrors(uint32_t voltage)
{
	errorCode error = NO_ERRORS;

	if (voltage <= (GROUND_VOLTAGE + ERROR_MARGIN))
	{
		error = SHORT_TO_GROUND;
	}

	if (voltage >= (VCC_VOLTAGE - ERROR_MARGIN))
	{
		error = SHORT_TO_VCC;
	}

	return error;
}

/** checkPressureSenorForErrors
 * @brief Function checks pressure sensor for errors such as under pressure and over pressure.
 * 
 * @param Signal *pressureSensor - pointer to the Signal structure.
 *
 * @retval if any errors occurred - return error id, if none - return NO_ERRORS.
 **/
errorCode checkPressureSenorForErrors(Signal *pressureSensor)
{
	errorCode error = NO_ERRORS;
	uint32_t value = *(pressureSensor->raw_value_ptr);

	if (value <= pressureSensor->min)
	{
		error = UNDER_PRESSURE;
	}

	if (value >= pressureSensor->max)
	{
		error = OVER_PRESSURE;
	}

	return error;
}

/*
 *
 * Functions below provide wrappers for errors that will be transmitted via CAN to the MCU_Main.
 *
 * */
void BrakeSensorShortCircuitToGroundError_Handler(void)
{
	CAN_ReportError(Error_BrakeSensorShortCircuitToGround_ID);
}

void BrakeSensorShortCircuitToVccError_Handler(void)
{
	CAN_ReportError(Error_BrakeSensorShortCircuitToVcc_ID);
}

void BrakeSensorShortCircuitBetweenSignalLinesError_Handler(void)
{
	CAN_ReportError(Error_BrakeSensorShortCircuitBetweenSignalLines_ID);
}

void BrakeSensorOutOfRangeError_Handler(void)
{
	CAN_ReportError(Error_BrakeSensorOutOfRange_ID);
}

void BrakeSensorDeviationBetweenTwoSignalsError_Handler(void)
{
	CAN_ReportError(Error_BrakeSensorDeviationBetweenTwoSignals_ID);
}

void AcceleratorSensorShortCircuitToGroundError_Handler(void)
{
	CAN_ReportError(Error_AcceleratorSensorShortCircuitToGround_ID);
}

void AcceleratorSensorShortCircuitToVccError_Handler(void)
{
	CAN_ReportError(Error_AcceleratorSensorShortCircuitToVcc_ID);
}

void AcceleratorSensorShortCircuitBetweenSignalLinesError_Handler(void)
{
	CAN_ReportError(Error_AcceleratorSensorShortCircuitBetweenSignalLines_ID);
}

void AcceleratorSensorOutOfRangeError_Handler(void)
{
	CAN_ReportError(Error_AcceleratorSensorOutOfRange_ID);
}

void AcceleratorSensorDeviationBetweenTwoSignalsError_Handler(void)
{
	CAN_ReportError(Error_AcceleratorSensorDeviationBetweenTwoSignals_ID);
}

void SteeringWheelSensorShortCircuitToGroundError_Handler(void)
{
	CAN_ReportError(Error_SteeringWheelSensorShortCircuitToGround_ID);
}

void SteeringWheelSensorShortCircuitToVccError_Handler(void)
{
	CAN_ReportError(Error_SteeringWheelSensorShortCircuitToVcc_ID);
}

void RightWheelSensorShortCircuitToGroundError_Handler(void)
{
	CAN_ReportError(Error_RightWheelSensorShortCircuitToGround_ID);
}

void RightWheelSensorOutOfRangeError_Handler(void)
{
	CAN_ReportError(Error_RightWheelSensorOutOfRange_ID);
}

void LeftWheelSensorShortCircuitToGroundError_Handler(void)
{
	CAN_ReportError(Error_LeftWheelSensorShortCircuitToGround_ID);
}

void LeftWheelSensorOutOfRangeError_Handler(void)
{
	CAN_ReportError(Error_LeftWheelSensorOutOfRange_ID);
}

void PressureSensorUnderPressureError_Handler(void)
{
	CAN_ReportError(Error_UnderPressure_ID);
}
void PressureSensorOverPressureError_Handler(void)
{
	CAN_ReportError(Error_OverPressure_ID);
}

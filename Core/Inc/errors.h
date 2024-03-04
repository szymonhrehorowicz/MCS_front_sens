/**
 ******************************************************************************
 * @file           : errors.h
 * @brief          : Header for errors.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @author Szymon Hrehorowicz
 * PK MechPower 2022 - 2023
 ******************************************************************************
 */

#include "main.h"
#include "can_config.h"
#include "math.h"
#include "stdlib.h"

#define GROUND_VOLTAGE 0
#define VCC_VOLTAGE 4096
#define ERROR_DELAY 400
#define ERROR_MARGIN 120 // 120 corresponds to the 0.1V difference in the ADC reading

errorCode checkPositionSensorForErrors(Signal *signalA, Signal *signalB);
errorCode checkST_WHEELforErrors(uint32_t signal);
errorCode checkPressureSenorForErrors(Signal *pressureSensor);

void BrakeSensorShortCircuitToGroundError_Handler(void);
void BrakeSensorShortCircuitToVccError_Handler(void);
void BrakeSensorShortCircuitBetweenSignalLinesError_Handler(void);
void BrakeSensorOutOfRangeError_Handler(void);
void BrakeSensorDeviationBetweenTwoSignalsError_Handler(void);
void AcceleratorSensorShortCircuitToGroundError_Handler(void);
void AcceleratorSensorShortCircuitToVccError_Handler(void);
void AcceleratorSensorShortCircuitBetweenSignalLinesError_Handler(void);
void AcceleratorSensorOutOfRangeError_Handler(void);
void AcceleratorSensorDeviationBetweenTwoSignalsError_Handler(void);
void SteeringWheelSensorShortCircuitToGroundError_Handler(void);
void SteeringWheelSensorShortCircuitToVccError_Handler(void);
void RightWheelSensorShortCircuitToGroundError_Handler(void);
void RightWheelSensorShortCircuitToVccError_Handler(void);
void RightWheelSensorOutOfRangeError_Handler(void);
void LeftWheelSensorShortCircuitToGroundError_Handler(void);
void LeftWheelSensorShortCircuitToVccError_Handler(void);
void LeftWheelSensorOutOfRangeError_Handler(void);
void PressureSensorUnderPressureError_Handler(void);
void PressureSensorOverPressureError_Handler(void);


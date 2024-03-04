/*
 * can_config.h
 *
 *  Created on: 27.02.2022
 * Modified on: 30.07.2022
 *      Author: Krystian Sosin
 *     Version: 1.0.1
 * Last change: Fix minor bugs and add CAN_AcknowledgeWriteMessage() function.
 */

#ifndef INC_CAN_CONFIG_H_
#define INC_CAN_CONFIG_H_

#include "main.h"
#include "stdbool.h"
#include "can.h"
/* ---------------------------------------------------------------------------------------- */
/* -------------------------------START OF DEFINES----------------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* USER CODE BEGIN DEFINES */
#define NUMBER_OF_READ_REGS (4U)
#define NUMBER_OF_WRITE_REGS (2U)
#define NUMBER_OF_ERROR_REGS (18U)
/* USER CODE END DEFINES */

#define ERROR_DLC (2U)
#define ACKNOWLEDMENT_DLC (2U)
#define FIRST_ARRAY_ELEMENT (0U)

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------END OF DEFINES------------------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------START OF ENUMS DEFINITIONS------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* USER CODE BEGIN Registers */
/* Registers */
// All of Reg1 - RegN should be replaced by meaningful name of reg (don't delete prefixs)
typedef enum
{
	Read_CalibratedValue_ID = 0x01u,
	Read_AnalogSensorValues_ID = 0x03u,	 // BPPS, APPS, ST_WHEEL
	Read_DigitalSensorValues_ID = 0x04u, // left and right wheels speed
	Read_PressureSensorValues_ID = 0x05u,
} ReadRegsID;

typedef enum
{
	Write_CalibrationState_ID = 0x02u,
	Write_Handshake_ID = 0xffu,
} WriteRegsID;

typedef enum
{
	Error_BrakeSensorShortCircuitToGround_ID = 0x01u,
	Error_BrakeSensorShortCircuitToVcc_ID = 0x02u,
	Error_BrakeSensorShortCircuitBetweenSignalLines_ID = 0x03u,
	Error_BrakeSensorOutOfRange_ID = 0x04u,
	Error_BrakeSensorDeviationBetweenTwoSignals_ID = 0x05u,
	Error_AcceleratorSensorShortCircuitToGround_ID = 0x06u,
	Error_AcceleratorSensorShortCircuitToVcc_ID = 0x07u,
	Error_AcceleratorSensorShortCircuitBetweenSignalLines_ID = 0x08u,
	Error_AcceleratorSensorOutOfRange_ID = 0x09u,
	Error_AcceleratorSensorDeviationBetweenTwoSignals_ID = 0x0Au,
	Error_SteeringWheelSensorShortCircuitToGround_ID = 0x0Bu,
	Error_SteeringWheelSensorShortCircuitToVcc_ID = 0x0Cu,
	Error_RightWheelSensorShortCircuitToGround_ID = 0x0Du,
	Error_RightWheelSensorOutOfRange_ID = 0x0Eu,
	Error_LeftWheelSensorShortCircuitToGround_ID = 0x0Fu,
	Error_LeftWheelSensorOutOfRange_ID = 0x10u,
	Error_UnderPressure_ID = 0x11u,
	Error_OverPressure_ID = 0x12u,
} ErrorRegsID;
/* USER CODE END Registers */

/* Enums to avoid magic numbers */
typedef enum
{
	ReadMessage,
	ReadRegID
} ReadFrameID;

typedef enum
{
	ResponseRegID,
	ResponseData1,
	ResponseData2,
	ResponseData3,
	ResponseData4,
	ResponseData5,
	ResponseData6
} ResponseFrame;

typedef enum
{
	WriteMessage_reg,
	WriteState,
} WriteFrame;

typedef enum
{
	ErrorMessage_reg,
	ErrorRegID
} ErrorFrame;

typedef enum
{
	AcknowledgmentMessage_reg,
	WriteRegID
} AcknowledgmentFrame;

typedef enum
{
	Error_ReportMessage = 0x1Du,
	Read_RequestMessage = 0x3Du,
	Write_AcknowledgmentMessage = 0x5Du
} CANStandardMessage;

/* USER CODE BEGIN NodeAddress */
typedef enum
{
	Rx_ID = 0x005A,
	Tx_ID = 0x005F
} NodeAddress;

enum
{
	CALIBRATED_SENSOR = 0u,
	CALIBRATION_STATE = 0u,
	ANALOG_SENSORS = 1u,
	APPS = 1u,
	BPPS = 1u,
	ST_WHEEL = 1u,
	DIGITAL_SENSORS = 2u,
	R_WHEEL_SENSOR = 2u,
	L_WHEEL_SENSOR = 2u,
	PRESSURE_SENSORS = 3u,
	FIRST_PRESSURE_SENSOR = 3u,
	SECOND_PRESSURE_SENSOR = 3u,
};
/* USER CODE END NodeAddress */

/* ---------------------------------------------------------------------------------------- */
/* -------------------------------END OF ENUMS DEFINITIONS--------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------START OF VARIABLES DEFINITIONS------------------------------ */
/* ---------------------------------------------------------------------------------------- */

typedef void (*ReadReactionHandlerFuncPtr)(void);
typedef void (*WriteReactionHandlerFuncPtr)(void);

typedef struct
{
	WriteRegsID Write_RegID;
	WriteReactionHandlerFuncPtr Write_ReactionHandler;
	bool *Write_State;
} WriteMessageFrame;

typedef struct
{
	uint8_t Response_DLC;
	ReadRegsID Response_RegID;
	ReadReactionHandlerFuncPtr Read_ReactionHandler;
	uint8_t *Response_Data1;
	uint8_t *Response_Data2;
	uint8_t *Response_Data3;
	uint8_t *Response_Data4;
	uint8_t *Response_Data5;
	uint8_t *Response_Data6;
} ResponseMessageFrame;

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------END OF STRUCTURES DEFINITIONS-------------------------------- */
/* ---------------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------START OF FUNCTIONS DECLARATIONS------------------------------ */
/* ---------------------------------------------------------------------------------------- */

void CAN_Init(void);
void CAN_On_Receive(uint8_t *RxData);
void CAN_Receive(CAN_HandleTypeDef *CANPointer, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData);
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxDLC, uint8_t *TxData, uint32_t *TxMailbox);
void CAN_Respond(void);
void CAN_ProcessWriteCommand(void);
void CAN_AcknowledgeWriteMessage(WriteRegsID WriteReqID);
void CAN_ReportError(ErrorRegsID ErrorID);
void CANBUS_Error_Handler(void);
void Read_CalibratedValue_Handler(void);
void Read_AnalogSensors_Handler(void);
void Read_DigitalSensors_Handler(void);
void Read_PressureSensors_Handler(void);
void Write_CalibrationState_Handler(void);
void Write_Handshake_Handler(void);

/* ---------------------------------------------------------------------------------------- */
/* ---------------------------END OF FUNCTIONS DECLARATIONS-------------------------------- */
/* ---------------------------------------------------------------------------------------- */

#endif /* INC_CAN_CONFIG_H_ */

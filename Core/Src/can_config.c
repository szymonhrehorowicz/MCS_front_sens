/*
 * can_config.c
 *
 *   Created on: 27.02.2022
 *  Modified on: 30.07.2022
 *       Author: Krystian Sosin
 *      Version: 1.0.2
 *  Last change: Fix minor bugs and add CAN_AcknowledgeWriteMessage() function.
 */

#include "main.h"
#include "can_config.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_can.h"
#include "stdbool.h"

/* Variables and arrays needed to proper work of CAN */
CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* USER CODE BEGIN Externs */

/* Externs used in configs */
extern bool Write_isInCalibrationState;

extern uint8_t Read_CurrentCalibratedSensorValue[2];

extern uint8_t Read_BPPSPosition[2];
extern uint8_t Read_APPSPosition[2];

extern uint8_t Read_ST_WHEELAngle[2];

extern uint8_t Read_R_WHEELSpeed[2];
extern uint8_t Read_L_WHEELSpeed[2];

extern uint8_t Read_PressureOneValue[2];
extern uint8_t Read_PressureTwoValue[2];

extern uint8_t calibrationStage;
/* USER CODE END Externs */

/**
 * CAN READ MESSAGE FRAME
 *
 * | RxID  |  2  | 0x3D | ADDR  |
 * | StdID | DLC | READ | RegID |
 *
 *
 */
// Enum of ReadRegs defined in can_config.h. Nothing needs to be configured here.
/* USER CODE BEGIN ResponseMessage */

/**
 *  CAN RESPONSE MESSAGE FRAME
 *
 * | TxID  | DLC | ADDR  | VALUE  | ... | VALUE  |
 * | StdID | DLC | RegID | DATA_1 | ... | DATA_N |
 *
 **/
ResponseMessageFrame ResponseMessage[NUMBER_OF_READ_REGS] = {
	{
		.Response_DLC = 3u,															 // Data length of response message
		.Read_ReactionHandler = Read_CalibratedValue_Handler,						 // Handler of reaction to read request from MCU
		.Response_RegID = Read_CalibratedValue_ID,									 // Address of regs which response refers
		.Response_Data1 = &Read_CurrentCalibratedSensorValue[MOST_SIGNIFICANT_BYTE], // Returned data to MCU
		.Response_Data2 = &Read_CurrentCalibratedSensorValue[LESS_SIGNIFICANT_BYTE],
	},
	{
		.Response_DLC = 7u,
		.Read_ReactionHandler = Read_AnalogSensors_Handler,
		.Response_RegID = Read_AnalogSensorValues_ID,
		.Response_Data1 = &Read_BPPSPosition[MOST_SIGNIFICANT_BYTE],
		.Response_Data2 = &Read_BPPSPosition[LESS_SIGNIFICANT_BYTE],
		.Response_Data3 = &Read_APPSPosition[MOST_SIGNIFICANT_BYTE],
		.Response_Data4 = &Read_APPSPosition[LESS_SIGNIFICANT_BYTE],
		.Response_Data5 = &Read_ST_WHEELAngle[MOST_SIGNIFICANT_BYTE],
		.Response_Data6 = &Read_ST_WHEELAngle[LESS_SIGNIFICANT_BYTE],
	},
	{
		.Response_DLC = 5u,
		.Read_ReactionHandler = Read_DigitalSensors_Handler,
		.Response_RegID = Read_DigitalSensorValues_ID,
		.Response_Data1 = &Read_R_WHEELSpeed[MOST_SIGNIFICANT_BYTE],
		.Response_Data2 = &Read_R_WHEELSpeed[LESS_SIGNIFICANT_BYTE],
		.Response_Data3 = &Read_L_WHEELSpeed[MOST_SIGNIFICANT_BYTE],
		.Response_Data4 = &Read_L_WHEELSpeed[LESS_SIGNIFICANT_BYTE],
	},
	{
		.Response_DLC = 5u,
		.Read_ReactionHandler = Read_PressureSensors_Handler,
		.Response_RegID = Read_PressureSensorValues_ID,
		.Response_Data1 = &Read_PressureOneValue[MOST_SIGNIFICANT_BYTE],
		.Response_Data2 = &Read_PressureOneValue[LESS_SIGNIFICANT_BYTE],
		.Response_Data3 = &Read_PressureTwoValue[MOST_SIGNIFICANT_BYTE],
		.Response_Data4 = &Read_PressureTwoValue[LESS_SIGNIFICANT_BYTE],
	},
};
/* USER CODE END ResponseMessage */

/* USER CODE BEGIN WriteMessage */

/**
 *  CAN WRITE MESSAGE FRAME
 *
 * | RxID  | DLC | ADDR  | VALUE  | ... | VALUE  |
 * | StdID | DLC | WRITE | DATA_1 | ... | DATA_N |
 *
 **/
WriteMessageFrame WriteMessage[NUMBER_OF_WRITE_REGS] = {
	// Stop Calibration Stage
	{
		.Write_RegID = Write_CalibrationState_ID,				 // Reg which should be written by MCU command
		.Write_ReactionHandler = Write_CalibrationState_Handler, // Handler of reaction to write request from MCU
		.Write_State = &Write_isInCalibrationState,				 // If this MCU command should change state of sth this pointer should point to variable which regards this state eg. if MCU want to light up brake light, this structure element should point to variable which contain the state of brake lights
	},
	// Handshake with MCS_Main
	{
		.Write_RegID = Write_Handshake_ID,
		.Write_ReactionHandler = Write_Handshake_Handler,
	}
};
/* USER CODE END WriteMessage */
/**
 *  CAN ERROR MESSAGE FRAME
 *
 * | TxID  |  2  | 0x1D  |    ID   |
 * | StdID | DLC | ERROR | ErrorID |
 *
 */
// Enum of ErrorRegs defined in can_config.h. Nothing needs to be configured here.
/** CAN_Init
 * @brief Function to ensure proper work of CAN interface
 - configuration of filter and calling essantial functions of CAN initialization
 Filter configured in accordance with E&S Team Project Guidlines.
 *
 * @retval None.
 **/
void CAN_Init(void)
{
	sFilterConfig.FilterBank = 1;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = Rx_ID << 5;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0xFFFF << 5;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	TxHeader.StdId = Tx_ID;
	TxHeader.ExtId = 0x0000;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
}

/** HAL_CAN_RxFifo0MsgPendingCallback
 * @brief HAL Callback to handle interuption from CAN new message
 *
 * @param hcan pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 *
 * @retval None
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Receive(hcan, &RxHeader, RxData);
	CAN_On_Receive(RxData);
}

/** CAN_On_Receive
 * @brief Function to procces received message - checking if message is read or write command and call specific handler
 *
 * @param RxData pointer to uint8_t array which stores received data
 *
 * @retval None
 **/
void CAN_On_Receive(uint8_t *RxData)
{
	if (Read_RequestMessage == RxData[ReadMessage])
	{
		CAN_Respond();
	}
	else
	{
		CAN_ProcessWriteCommand();
	}
}

/** CAN_Receive
 * @brief Function to receive data via CAN and, if neccessary, report error of CAN connection
 *
 * @param CANPointerpointer to a CAN_HandleTypeDef structure that contains
 *                          the configuration information for the specified CAN.
 * @param RxHeader CAN_RxHeaderTypeDef pointer to structure that contains RxHeader configuration.
 * @param RxData uint8_t pointer to array that will contain received data.
 *
 * @retval None.
 **/
void CAN_Receive(CAN_HandleTypeDef *CANPointer, CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData)
{
	if (HAL_CAN_GetRxMessage(CANPointer, CAN_RX_FIFO0, RxHeader, RxData) != HAL_OK)
	{
		CANBUS_Error_Handler();
	}
};

/** CAN_Transmit
 * @brief Function to transmit data via CAN and, if neccessary, report error of CAN connection
 *
 * @param TxHeader CAN_TxHeaderTypeDef pointer to structure that contains TxHeader configuration.
 * @param TxDLC uint8_t variable which contains Date Length of CAN message.
 * @param TxData uint8_t pointer to array that contains data to transmit.
 * @param TxMailbox uint32_t pointer to array that contains whole CAN message to transmit.
 *
 * @retval None.
 **/
void CAN_Transmit(CAN_TxHeaderTypeDef *TxHeader, uint8_t TxDLC, uint8_t *TxData, uint32_t *TxMailbox)
{
	TxHeader->DLC = TxDLC;
	if (HAL_CAN_AddTxMessage(&hcan, TxHeader, TxData, TxMailbox) != HAL_OK)
	{
		CANBUS_Error_Handler();
	}
}

/** CAN_Respond
 * @brief Function to respond in connection with read request from MCU
 *
 * @retval None.
 **/
void CAN_Respond(void)
{
	for (int i = FIRST_ARRAY_ELEMENT; i < NUMBER_OF_READ_REGS; i++)
	{
		if (ResponseMessage[i].Response_RegID == RxData[ReadRegID])
		{
			ResponseMessage[i].Read_ReactionHandler();
		}
	}
}

/** CAN_ProcessWriteCommand
 * @brief Function to process write command
 *
 * @retval None.
 **/
void CAN_ProcessWriteCommand(void)
{
	for (int i = FIRST_ARRAY_ELEMENT; i < NUMBER_OF_WRITE_REGS; i++)
	{
		if (WriteMessage[i].Write_RegID == RxData[WriteMessage_reg])
		{
			CAN_AcknowledgeWriteMessage(WriteMessage[i].Write_RegID);
			WriteMessage[i].Write_ReactionHandler();
		}
	}
}

/** CAN_AcknowledgeWriteMessage
 * @brief Function to send acknowledment received write instruction via CAN
 *
 * @param WriteReqID ID of received write instruction
 *
 * @retval None.
 **/
void CAN_AcknowledgeWriteMessage(WriteRegsID WriteReqID)
{
	TxData[AcknowledgmentMessage_reg] = Write_AcknowledgmentMessage; // 1st Data Byte: Standard Write Acknowledgment instruction
	TxData[WriteRegID] = WriteReqID;								 // 2nd Data Byte: Acknowledged Received Write Command ReqID
	CAN_Transmit(&TxHeader, ACKNOWLEDMENT_DLC, TxData, &TxMailbox);	 // Transmit Data
}

/** CAN_ReportError
 * @brief Function to report error via CAN
 *
 * @param ErrorID ID of reported Error Register
 *
 * @retval None.
 **/
void CAN_ReportError(ErrorRegsID ErrorID)
{
	TxData[ErrorMessage_reg] = Error_ReportMessage;			// 1st Data Byte: Standard Error Report instruction
	TxData[ErrorRegID] = ErrorID;							// 2nd Data Byte: Reported Error ID
	CAN_Transmit(&TxHeader, ERROR_DLC, TxData, &TxMailbox); // Transmit Data
}

/* USER CODE BEGIN CANBUS_Error_Handler */

/** CANBUS_Error_Handler
 * @brief General error handler of CAN connection and communication
 *
 * @retval None.
 * */
void CANBUS_Error_Handler(void)
{
	__disable_irq();
	/*
	 Put here behaviour of ECU when error will be occured.
	 */
}

/* USER CODE BEGIN ReadReactionHandlers */

/** Read_CalibratedValue_Handler
 * @brief Function that sends to the MCU value of a sensor that is currently beeing calibrated.
 *
 * @param void
 *
 * @retval None.
 **/
void Read_CalibratedValue_Handler(void)
{
	calibrate();
	TxData[ResponseRegID] = ResponseMessage[CALIBRATED_SENSOR].Response_RegID;
	TxData[ResponseData1] = *(ResponseMessage[CALIBRATED_SENSOR].Response_Data1);
	TxData[ResponseData2] = *(ResponseMessage[CALIBRATED_SENSOR].Response_Data2);

	CAN_Transmit(&TxHeader, ResponseMessage[CALIBRATED_SENSOR].Response_DLC, TxData, &TxMailbox);
}

/** Read_AnalogSensors_Handler
 * @brief Function that sends to the MCU current value of APPS, BPPS and ST_WHEEL sensors.
 *
 * @param void
 *
 * @retval None.
 **/
void Read_AnalogSensors_Handler(void)
{
	TxData[ResponseRegID] = ResponseMessage[ANALOG_SENSORS].Response_RegID;
	TxData[ResponseData1] = *(ResponseMessage[BPPS].Response_Data1);
	TxData[ResponseData2] = *(ResponseMessage[BPPS].Response_Data2);
	TxData[ResponseData3] = *(ResponseMessage[APPS].Response_Data3);
	TxData[ResponseData4] = *(ResponseMessage[APPS].Response_Data4);
	TxData[ResponseData5] = *(ResponseMessage[ST_WHEEL].Response_Data5);
	TxData[ResponseData6] = *(ResponseMessage[ST_WHEEL].Response_Data6);

	CAN_Transmit(&TxHeader, ResponseMessage[ANALOG_SENSORS].Response_DLC, TxData, &TxMailbox);
}

/** Read_DigitalSensors_Handler
 * @brief Function that sends to the MCU current speed of both front wheel sensors.
 *
 * @param void
 *
 * @retval None.
 **/
void Read_DigitalSensors_Handler(void)
{
	TxData[ResponseRegID] = ResponseMessage[DIGITAL_SENSORS].Response_RegID;
	TxData[ResponseData1] = *(ResponseMessage[R_WHEEL_SENSOR].Response_Data1);
	TxData[ResponseData2] = *(ResponseMessage[R_WHEEL_SENSOR].Response_Data2);
	TxData[ResponseData3] = *(ResponseMessage[L_WHEEL_SENSOR].Response_Data3);
	TxData[ResponseData4] = *(ResponseMessage[L_WHEEL_SENSOR].Response_Data4);

	CAN_Transmit(&TxHeader, ResponseMessage[DIGITAL_SENSORS].Response_DLC, TxData, &TxMailbox);
}

/** Read_PressureSensors_Handler
 * @brief Function that sends to the MCU current pressure reading from both pressure sensors.
 *
 * @param void
 *
 * @retval None.
 **/
void Read_PressureSensors_Handler(void)
{
	TxData[ResponseRegID] = ResponseMessage[PRESSURE_SENSORS].Response_RegID;
	TxData[ResponseData1] = *(ResponseMessage[FIRST_PRESSURE_SENSOR].Response_Data1);
	TxData[ResponseData2] = *(ResponseMessage[FIRST_PRESSURE_SENSOR].Response_Data2);
	TxData[ResponseData3] = *(ResponseMessage[SECOND_PRESSURE_SENSOR].Response_Data3);
	TxData[ResponseData4] = *(ResponseMessage[SECOND_PRESSURE_SENSOR].Response_Data4);

	CAN_Transmit(&TxHeader, ResponseMessage[PRESSURE_SENSORS].Response_DLC, TxData, &TxMailbox);
}


/* USER CODE END ReadReactionHandlers */

/* USER CODE BEGIN WriteReactionHandlers */

/** Write_CalibrationState_Handler
 * @brief Function that stops the calibration proccess.
 *
 * @param void
 *
 * @retval None.
 **/
void Write_CalibrationState_Handler(void)
{
	*(WriteMessage[CALIBRATION_STATE].Write_State) = RxData[WriteState];
}

/** Write_Handshake_Handler
 * @brief Function that does nothing.
 *
 * @param void
 *
 * @retval None.
 **/
void Write_Handshake_Handler(void)
{

}
/* USER CODE END WriteReactionHandlers */

/* Includes ------------------------------------------------------------------*/
#include "usbd_iad_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_IAD
 * @brief usbd core module
 * @{
 */

/** @defgroup USBD_IAD_Private_TypesDefinitions
 * @{
 */
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */
/**
 * @}
 */

/** @defgroup USBD_IAD_Private_Defines
 * @{
 */
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over IAD */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024
/* USER CODE END PRIVATE_DEFINES */
/**
 * @}
 */

/** @defgroup USBD_IAD_Private_Macros
 * @{
 */
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
 * @}
 */

/** @defgroup USBD_IAD_Private_Variables
 * @{
 */

/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferHS1[APP_RX_DATA_SIZE];
uint8_t UserRxBufferHS2[APP_RX_DATA_SIZE];

/* Send Data over USB IAD are stored in this buffer       */
uint8_t UserTxBufferHS1[APP_TX_DATA_SIZE];
uint8_t UserTxBufferHS2[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
uint32_t UserRxLenHS1;
uint32_t UserRxLenHS2;
/* USER CODE END PRIVATE_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_IAD_IF_Exported_Variables
 * @{
 */
extern USBD_HandleTypeDef hUsbDeviceHS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
 * @}
 */

/** @defgroup USBD_IAD_Private_FunctionPrototypes
 * @{
 */

static int8_t IAD_Init_HS(void);
static int8_t IAD_DeInit_HS(void);
static int8_t IAD_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t IAD_Receive_CDC1_HS(uint8_t* pbuf, uint32_t *Len);
static int8_t IAD_Receive_CDC2_HS(uint8_t* pbuf, uint32_t *Len);
/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
 * @}
 */

USBD_IAD_ItfTypeDef USBD_IAD_Interface_fops_HS =
{
	IAD_Init_HS,
	IAD_DeInit_HS,
	IAD_Control_HS,
	IAD_Receive_CDC1_HS,
	IAD_Receive_CDC2_HS
};

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  IAD_Init_HS
 *         Initializes the IAD media low layer over the USB HS IP
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t IAD_Init_HS(void) {
	/* USER CODE BEGIN 8 */
	/* Set Application Buffers */
	USBD_IAD_CDC1_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS1, 0);
	USBD_IAD_CDC1_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS1);
	UserRxLenHS1 = 0;

	USBD_IAD_CDC2_SetTxBuffer(&hUsbDeviceHS, UserTxBufferHS2, 0);
	USBD_IAD_CDC2_SetRxBuffer(&hUsbDeviceHS, UserRxBufferHS2);
	UserRxLenHS2 = 0;
	return (USBD_OK);
	/* USER CODE END 8 */
}

/**
 * @brief  IAD_DeInit_HS
 *         DeInitializes the IAD media low layer
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t IAD_DeInit_HS(void) {
	/* USER CODE BEGIN 9 */
	return (USBD_OK);
	/* USER CODE END 9 */
}

/**
 * @brief  IAD_Control_HS
 *         Manage the IAD class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t IAD_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length) {
	/* USER CODE BEGIN 10 */
	switch (cmd) {
	case IAD_SEND_ENCAPSULATED_COMMAND:

		break;

	case IAD_GET_ENCAPSULATED_RESPONSE:

		break;

	case IAD_SET_COMM_FEATURE:

		break;

	case IAD_GET_COMM_FEATURE:

		break;

	case IAD_CLEAR_COMM_FEATURE:

		break;

		/*******************************************************************************/
		/* Line Coding Structure                                                       */
		/*-----------------------------------------------------------------------------*/
		/* Offset | Field       | Size | Value  | Description                          */
		/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
		/* 4      | bCharFormat |   1  | Number | Stop bits                            */
		/*                                        0 - 1 Stop bit                       */
		/*                                        1 - 1.5 Stop bits                    */
		/*                                        2 - 2 Stop bits                      */
		/* 5      | bParityType |  1   | Number | Parity                               */
		/*                                        0 - None                             */
		/*                                        1 - Odd                              */
		/*                                        2 - Even                             */
		/*                                        3 - Mark                             */
		/*                                        4 - Space                            */
		/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
		/*******************************************************************************/
	case IAD_SET_LINE_CODING:

		break;

	case IAD_GET_LINE_CODING:
		pbuf[0] = (uint8_t) (9600 >> 0);
		pbuf[1] = (uint8_t) (9600 >> 8);
		pbuf[2] = (uint8_t) (9600 >> 16);
		pbuf[3] = (uint8_t) (9600 >> 24);
		pbuf[4] = 0x00;
		pbuf[5] = 0x00;
		pbuf[6] = 0x08;
		break;

	case IAD_SET_CONTROL_LINE_STATE:

		break;

	case IAD_SEND_BREAK:

		break;

	default:
		break;
	}

	return (USBD_OK);
	/* USER CODE END 10 */
}

/**
 * @brief  IAD_Receive_HS
 *         Data received over USB OUT endpoint are sent over IAD interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         untill exiting this function. If you exit this function before transfer
 *         is complete on IAD interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t IAD_Receive_CDC1_HS(uint8_t* Buf, uint32_t *Len) {
	/* USER CODE BEGIN 11 */
	USBD_IAD_CDC1_SetRxBuffer(&hUsbDeviceHS, &Buf[*Len]);
	UserRxLenHS1 += *Len;
	USBD_IAD_CDC1_ReceivePacket(&hUsbDeviceHS);
	return (USBD_OK);
	/* USER CODE END 11 */
}

/**
 * @brief  IAD_Receive_HS
 *         Data received over USB OUT endpoint are sent over IAD interface
 *         through this function.
 *
 *         @note
 *         This function will block any OUT packet reception on USB endpoint
 *         untill exiting this function. If you exit this function before transfer
 *         is complete on IAD interface (ie. using DMA controller) it will result
 *         in receiving more data while previous ones are still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t IAD_Receive_CDC2_HS(uint8_t* Buf, uint32_t *Len) {
	/* USER CODE BEGIN 11 */
	USBD_IAD_CDC2_SetRxBuffer(&hUsbDeviceHS, &Buf[*Len]);
	UserRxLenHS2 += *Len;
	USBD_IAD_CDC2_ReceivePacket(&hUsbDeviceHS);
	return (USBD_OK);
	/* USER CODE END 11 */
}

/**
 * @brief  IAD_Transmit_HS
 *         Data send over USB IN endpoint are sent over IAD interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be send
 * @param  Len: Number of data to be send (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t IAD_CDC1_Transmit_HS(uint8_t* Buf, uint16_t Len) {
	uint8_t result = USBD_OK;

	/* USER CODE BEGIN 12 */
	USBD_IAD_HandleTypeDef *hiad =
			(USBD_IAD_HandleTypeDef*) hUsbDeviceHS.pClassData;

	if (hiad->cdc1.TxState != 0) {
		return USBD_BUSY;
	}
	USBD_IAD_CDC1_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
	result = USBD_IAD_CDC1_TransmitPacket(&hUsbDeviceHS);
	/* USER CODE END 12 */
	return result;
}

/**
 * @brief  IAD_Transmit_HS
 *         Data send over USB IN endpoint are sent over IAD interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be send
 * @param  Len: Number of data to be send (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t IAD_CDC2_Transmit_HS(uint8_t* Buf, uint16_t Len) {
	uint8_t result = USBD_OK;

	/* USER CODE BEGIN 12 */
	USBD_IAD_HandleTypeDef *hiad =
			(USBD_IAD_HandleTypeDef*) hUsbDeviceHS.pClassData;

	if (hiad->cdc2.TxState != 0) {
		return USBD_BUSY;
	}
	USBD_IAD_CDC2_SetTxBuffer(&hUsbDeviceHS, Buf, Len);
	result = USBD_IAD_CDC2_TransmitPacket(&hUsbDeviceHS);
	/* USER CODE END 12 */
	return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
int8_t CDC_Receive_VCP1_HS(uint8_t* Buf, uint32_t *Len) {
	if (*Len > UserRxLenHS1)
		*Len = UserRxLenHS1;
	if (*Len > 0) {
		memcpy(Buf, UserRxBufferHS1, *Len);

		uint32_t bytesRemaining = UserRxLenHS1 - *Len;
		if (bytesRemaining != 0)
			memcpy(UserRxBufferHS1, &UserRxBufferHS1[*Len], bytesRemaining);

		UserRxLenHS1 = bytesRemaining;
		USBD_IAD_CDC1_SetRxBuffer(&hUsbDeviceHS, &UserRxBufferHS1[UserRxLenHS1]);
		USBD_IAD_CDC1_ReceivePacket(&hUsbDeviceHS);
	}
	return USBD_OK;
}

int8_t CDC_Receive_VCP2_HS(uint8_t* Buf, uint32_t *Len) {
	if (*Len > UserRxLenHS2)
		*Len = UserRxLenHS2;
	if (*Len > 0) {
		memcpy(Buf, UserRxBufferHS2, *Len);

		uint32_t bytesRemaining = UserRxLenHS2 - *Len;
		if (bytesRemaining != 0)
			memcpy(UserRxBufferHS2, &UserRxBufferHS2[*Len], bytesRemaining);

		UserRxLenHS2 = bytesRemaining;
		USBD_IAD_CDC2_SetRxBuffer(&hUsbDeviceHS, &UserRxBufferHS2[UserRxLenHS2]);
		USBD_IAD_CDC2_ReceivePacket(&hUsbDeviceHS);
	}
	return USBD_OK;
}

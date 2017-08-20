/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          :
  ******************************************************************************
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "usbd_cdc_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CDC 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1024
#define APP_TX_DATA_SIZE  1024
/* TIM handler declaration */
TIM_HandleTypeDef    TimHandle;
/* USER CODE END PRIVATE_DEFINES */
/**
  * @}
  */ 

/** @defgroup USBD_CDC_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
USBD_CDC_LineCodingTypeDef LineCoding = {
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_Variables
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/* Received Data over USB are stored in this buffer       */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserRxBuffer[APP_RX_DATA_SIZE];

/* Send Data over USB CDC are stored in this buffer       */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
uint32_t UserTxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
uint32_t UserTxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */
uint32_t UserRxBufPtrIn = 0;/* Increment this pointer or roll it back to
                               start address when data are received over USART */
uint32_t UserRxBufPtrOut = 0; /* Increment this pointer or roll it back to
                                 start address when data are sent over USB */
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_IF_Exported_Variables
  * @{
  */ 
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */


static void TIM_Config(void);

/**
  * @}
  */ 
  
/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */
static int8_t CDC_Init_FS     (void);
static int8_t CDC_DeInit_FS   (void);
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
volatile uint8_t DFU_STATE = 0;
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */ 
  
USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = 
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,  
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{ 
  /* USER CODE BEGIN 3 */ 

  TIM_Config();
  if(HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);


  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  UserTxBufPtrIn = 0;
  UserTxBufPtrOut = 0;
  UserRxBufPtrIn = 0;
  UserRxBufPtrOut = 0;
  //USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 3 */ 
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */ 
  return (USBD_OK);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  /* USER CODE BEGIN 5 */
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
 
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
 
    break;

  case CDC_SET_COMM_FEATURE:
 
    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

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
  case CDC_SET_LINE_CODING:   
    LineCoding.bitrate = (uint32_t) (pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
    LineCoding.format = pbuf[4];
    LineCoding.paritytype = pbuf[5];
    LineCoding.datatype = pbuf[6];
    check_DFU(LineCoding.bitrate);
    break;

  case CDC_GET_LINE_CODING:     
    pbuf[0] = (uint8_t)(LineCoding.bitrate);
    pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
    pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
    pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
    pbuf[4] = LineCoding.format;
    pbuf[5] = LineCoding.paritytype;
    pbuf[6] = LineCoding.datatype; 
    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:
 
    break;    
    
  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

void check_DFU(uint32_t bitrate){
  switch(DFU_STATE){
    case 0:
      if(bitrate == 230400)
        DFU_STATE++;
      else
        DFU_STATE = 0;
      break;
    case 1:
      if(bitrate == 38400)
        DFU_STATE++;
      else
        DFU_STATE = 0;
      break;
    case 2:
      DFU_STATE = 0;
      if(bitrate == 9600){
        dfu_run_bootloader();
      }
      break;
    default:
      DFU_STATE = 0;
  }
}



/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USB_Receive_FS (Buf, Len);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);/*
  int i = 0;
    
  for (i = 0; i < *Len; i++)
      UserTxBufferFS[i] = UserRxBufferFS[i];
  
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, &UserTxBufferFS[0], *Len);
  USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &UserRxBufferFS[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);*/
  return (USBD_OK);
  /* USER CODE END 6 */ 
}

/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t USB_write(uint8_t* Buf, uint16_t Len)
{
  // In case Buf cant fit into remaining TxBuffer
  uint16_t counter = 0; 
  while(counter < Len){
    UserTxBufferFS[UserTxBufPtrIn++] = Buf[counter++];
    
    /* To avoid buffer overflow */
    if(UserTxBufPtrIn == APP_TX_DATA_SIZE)
    {
      UserTxBufPtrIn = 0;
    }
  }
  
  return USBD_OK;
}



uint8_t USB_read(uint8_t *Buf, uint32_t *Len){
  uint32_t buffsize = 0;

  if(UserRxBufPtrOut != UserRxBufPtrIn){
    if(UserRxBufPtrOut > UserRxBufPtrIn){
      // UserRxBufPtrIn rolled over
      buffsize = APP_RX_DATA_SIZE - UserRxBufPtrOut;
    }else{
      buffsize = UserRxBufPtrIn - UserRxBufPtrOut;
    }
    if(buffsize > *Len){
      buffsize = *Len;
    }

    memcpy(Buf, (uint8_t*)&UserRxBuffer[UserRxBufPtrOut], buffsize);

    UserRxBufPtrOut += buffsize;
    if(UserRxBufPtrOut == APP_RX_DATA_SIZE){
      UserRxBufPtrOut = 0;          
    }
  } 
  *Len = buffsize;
    
     /*

     return (int)buffsize;*/
  /*if (*Len > UserRxBufPtrIn){
    *Len = UserRxBufPtrIn;
  }
  if (*Len > 0) {
    memcpy(Buf, UserRxBufferFS, *Len);

    uint32_t bytesRemaining = UserRxBufPtrIn - *Len;
    if (bytesRemaining != 0){
      memcpy(UserRxBufferFS, &UserRxBufferFS[*Len], bytesRemaining);
    }

    UserRxBufPtrIn = bytesRemaining;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &UserRxBufferFS[UserRxBufPtrIn]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }*/
  return USBD_OK;
}



uint32_t USB_available(void){
     uint32_t buffsize = 0;
     if(UserRxBufPtrOut != UserRxBufPtrIn){
          if(UserRxBufPtrOut > UserRxBufPtrIn){
               buffsize = APP_RX_DATA_SIZE - UserRxBufPtrOut + UserRxBufPtrIn;
          }else{
               buffsize = UserRxBufPtrIn - UserRxBufPtrOut;
          }
     }
     return buffsize;
}

/*
 *
 */

/*
 * This routine is called by CDC_Receive_FS() which is a callback
 * from the underlying USB Device servicing code which gets called
 * when there is data received on the USB endpoint for the device.
 * This routine therefore acts like a circular buffer DMA for data
 * received on the USB port similar to the behavior of the circulat
 * buffer implemented on the UART receive DMA when a UART is used
 * for console I/O
 */

void USB_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
     uint32_t counter = 0;
     while(counter < *Len){
          UserRxBuffer[UserRxBufPtrIn++] = Buf[counter++];
          if(UserRxBufPtrIn == APP_RX_DATA_SIZE){
               UserRxBufPtrIn  = 0;
          }
     }
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief  TIM period elapsed callback
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3){    
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0){
      return;
    }
    
    uint32_t buffsize;
    
    if(UserTxBufPtrOut != UserTxBufPtrIn){
      if(UserTxBufPtrOut > UserTxBufPtrIn){
        buffsize = APP_TX_DATA_SIZE - UserTxBufPtrOut;
      }else{
        buffsize = UserTxBufPtrIn - UserTxBufPtrOut;
      }
      
      
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBufferFS[UserTxBufPtrOut], buffsize);
      
      if(USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK){
        UserTxBufPtrOut += buffsize;
        if (UserTxBufPtrOut == APP_TX_DATA_SIZE)
        {
          UserTxBufPtrOut = 0;
        }
      }
    }
  }
}


/**
  * @brief  TIM_Config: Configure TIMx timer
  * @param  None.
  * @retval None.
  */
static void TIM_Config(void){  

  __HAL_RCC_TIM3_CLK_ENABLE();
    

  /* Set TIMx instance */
  TimHandle.Instance = TIM3;

  TimHandle.Init.Period = (CDC_POLLING_INTERVAL*20) - 1;   // 4999 
  TimHandle.Init.Prescaler = ((SystemCoreClock/2)/10000) - 1;//1; // ((72MHz/2)/10000)-1 = 
  TimHandle.Init.ClockDivision = 0U;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;

  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK){
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


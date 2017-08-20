/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * Copyright (c) 2016 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "sys.h"
#include "gpio.h"
#include "usb_device.h"
#include "usbd_iad_if.h"

/* USER CODE BEGIN Includes */
//#include "ros.h"
//#include <std_msgs/String.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//ros::NodeHandle nh;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void _Error_Handler(char *, int);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

*/
char hello[15] = "hello world!\r\n";
/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USB_DEVICE_Init();
	HAL_Delay(1000);


	/*nh.initNode();
  	nh.advertise(chatter);

  	uint8_t msg2[16];
	uint32_t totalLen;
	uint8_t result;
*/
	uint32_t tickstart = HAL_GetTick();
	//uint32_t tickstart2 = HAL_GetTick();
	while (1) {
		if((HAL_GetTick() - tickstart) > 100){
			//str_msg.data = hello;
	    	//chatter.publish(&str_msg);
	    	tickstart = HAL_GetTick();
	    	IAD_CDC1_Transmit_HS((uint8_t*)hello, 15);
		}

		/*if((HAL_GetTick() - tickstart2) > 100){
			totalLen = 16;
			msg2[0] = '.';
			msg2[1] = 'h';
			msg2[2] = 'e';
			msg2[3] = 'l';
			msg2[4] = 'l';
			msg2[5] = 'o';
			msg2[6] = '\r';
			msg2[7] = '\n';
			//result = CDC_Receive_VCP2_HS(msg2, &totalLen);
			//if (result == USBD_OK && totalLen != 0)
			//IAD_CDC2_Transmit_HS(msg2, totalLen);
			tickstart2 = HAL_GetTick();
		}*/
	    
		//nh.spinOnce();
	}
	/* USER CODE END 3 */
}

/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
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
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "hal.h"
#include "sys.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "led.h"
#include "usbd_cdc_if.h"
//#include "usart.h"
#include "usb_device.h"
#include "unit_tests.h"

/* USER CODE BEGIN Includes */
#include "ros.h"
#include <std_msgs/String.h>

//#include "OLED.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
ros::NodeHandle nh;
/*OLED *displayR;
OLED *displayL;
*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void _Error_Handler(char *, int);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

/* USER CODE END 0 */

int main(void)
{

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
  //MX_SPI1_Init();
  //MX_I2C1_Init();
  //MX_I2C2_Init();
  //MX_USART2_UART_Init();
  //MX_LED_Init();
  //MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  nh.initNode();
  nh.advertise(chatter);
/*
  int DISPLAYR_ADDR = 61;
  int DISPLAYL_ADDR = 60;

  // Initialize OLED displays
  displayR = new OLED();
  if(displayR->init(DISPLAYR_ADDR, 128, 64)){
    displayR->begin();
    displayR->display();
  }else{
    nh.logwarn("Right OLED display not detected!");
  }

  displayL = new OLED();
  if(displayL->init(DISPLAYL_ADDR, 128, 64)){
    displayL->begin();
    displayL->display();
  }else{
    nh.logwarn("Left OLED display not detected!");
  }*/

    
  while(1){
    str_msg.data = hello;
    chatter.publish( &str_msg );
    nh.spinOnce();
    HAL_Delay(1000);
    //test_USB_LOOP();
  }
  /* USER CODE END 3 */
/*
  displayR->close();
  displayL->close();
*/
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : dfu.c
  * Description        : This file provides code for jumping to the
                         built-in DFU bootloader.
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
#include "dfu.h"
#include "led.h"
/*
#define RESET_TO_BOOTLOADER_MAGIC_CODE (0xDEADBEEF)
#define SYSMEM_RESET_VECTOR (0x1fffC804)
#define BOOTLOADER_STACK_POINTER (0x20002250)

/* USER CODE BEGIN 1 */
//uint32_t dfu_reset_to_bootloader_magic;
/* USER CODE END 1 */
/*
void __initialize_hardware_early(void)
{
    if (*((unsigned long *)0x20003FF0) == RESET_TO_BOOTLOADER_MAGIC_CODE) {
        *((unsigned long *)0x20003FF0) = 0xCAFEFEED;
        __set_MSP(BOOTLOADER_STACK_POINTER);
        void (*SysMemBootJump)(void);
        SysMemBootJump = (void (*)(void)) (*((uint32_t *) SYSMEM_RESET_VECTOR));
        SysMemBootJump();
        while (1);
    } else {
        SystemInit();
    }
}*/

void dfu_run_bootloader()
{
    *((unsigned long *)0x20003FF0) = 0xDEADBEEF;
    NVIC_SystemReset();
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MPU9250_
#define __MPU9250_



/* Includes ------------------------------------------------------------------*/
#include <stm32f10x.h>
#include <stdio.h>


#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void gyro_data_ready_cb(void);

void initialize_mpu(void);
void update_mpu(void);


#ifdef __cplusplus
}
#endif

#endif /* __MPU9250_ */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

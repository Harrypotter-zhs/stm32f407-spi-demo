/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */
//#define SPI1_FLASH_CS_LOW()       HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
//#define SPI1_FLASH_CS_HIGH()      HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)
/* NORFLASH 片选信号 */
//#define NANDFLASH_CS(x)      do{ x ? \
//                                  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET) : \
//                                  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); \
//                            }while(0)
	
#define SYNC_FRAME            0xAA    // 同步帧
#define PACKET_SIZE           (128) // 每个数据包的大小
#define DELAY_BETWEEN_PACKETS 100   // 每个数据包之间的延时时间（单位：毫秒）	
//#define SPI_RX_NUM 		17                         /* 定义最大接收字节数 200 */
//#define SPI_REC_LEN   	12                     
#define SPI_RXBUFFERSIZE    1                       /* 缓存大小 */
extern uint8_t spi_rx_buffer[SPI_RXBUFFERSIZE];       /* HAL库USART接收Buffer */
extern uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t spi1_read_write_byte(uint8_t txdata);
void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size);														
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */


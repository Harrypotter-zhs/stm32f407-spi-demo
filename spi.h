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
#define NANDFLASH_CS(x)      do{ x ? \
                                  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET) : \
                                  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); \
                            }while(0)
	

//#define SPI_RX_NUM 		17                         /* 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟街斤拷锟斤拷 200 */
//#define SPI_REC_LEN   	12                     
#define SPI_RXBUFFERSIZE    1                       /* DMA中断接收每次中断接收的字节长度? */
extern uint8_t spi_rx_buffer[SPI_RXBUFFERSIZE];       /* HAL的SPI的DMA中断接收Buffer */
//extern uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
/* USER CODE END Private defines */
#define SYNC_FRAME            0xAA    // 帧同步吗
#define PACKET_SIZE           (128) // 帧包数据大小
#define SPI_RX_LEN     (PACKET_SIZE+10) //帧的总长度
typedef struct {
  uint8_t head[2];
  uint8_t index[2];// 原来是uint16_t
  uint8_t data_len[2];
  uint8_t all_data_len[2];// 原来是uint16_t,拆分为2个uint8_t
  uint8_t crc_data[2];
  uint8_t info[PACKET_SIZE];
}spi_send;
extern spi_send *spi_send_buf;
extern spi_send *spi_receive_buf;

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t spi1_read_write_byte(uint8_t txdata);
//void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size);				
//void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size, uint8_t count);
void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size);
//void SPI_MasterSendData_DMA_2(uint8_t* data, uint16_t data_size, uint32_t one_size, uint8_t times);
void SPI_MasterSendData_DMA_2(uint8_t* data, uint16_t data_size, uint32_t one_size, uint8_t times, uint8_t device_num);
void SPI_MasterReceiveData_DMA_2(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */


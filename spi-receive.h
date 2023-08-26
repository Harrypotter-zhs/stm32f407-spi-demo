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
/* NORFLASH Æ¬Ñ¡ÐÅºÅ */
//#define NANDFLASH_CS(x)      do{ x ? \
//                                  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET) : \
//                                  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); \
//                            }while(0)
	

//#define SPI_RX_NUM 		17                         /* ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö½ï¿½ï¿½ï¿½ 200 */
//#define SPI_REC_LEN   	12                     
#define SPI_RXBUFFERSIZE    1                       /* ï¿½ï¿½ï¿½ï¿½ï¿½Ð? */
//extern uint8_t spi_rx_buffer[SPI_RXBUFFERSIZE];       /* HALï¿½ï¿½USARTï¿½ï¿½ï¿½ï¿½Buffer */
//extern uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
#define SYNC_FRAME            0xAA    // Ö¡Í¬²½Âð
#define PACKET_SIZE           (128) // Ö¡°üÊý¾Ý´óÐ¡
#define SPI_RX_LEN     (PACKET_SIZE+10) //Ö¡µÄ×Ü³¤¶È
typedef struct {
  uint8_t head[2];
  uint8_t index[2];// Ô­À´ÊÇuint16_t
  uint8_t data_len[2];
  uint8_t all_data_len[2];// Ô­À´ÊÇuint16_t,²ð·ÖÎª2¸öuint8_t
  uint8_t crc_data[2];
  uint8_t info[PACKET_SIZE];
}spi_send;

extern spi_send *spi_send_buf;
extern spi_send *spi_rx_buf;

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t spi1_read_write_byte(uint8_t txdata);
//void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size);				
//void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size, uint8_t count);
void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size);
void SPI_MasterSendData_DMA_2(uint8_t* data, uint16_t data_size, uint32_t one_size, uint8_t times);
void SPI_MasterReceiveData_DMA_2(void);
void data_unpacking(uint8_t* data, uint32_t size);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */


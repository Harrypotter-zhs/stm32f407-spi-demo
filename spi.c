/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "crc16.h"
#include "string.h"
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */


//uint8_t SYNC_FRAME = 0xAA;    // 同步帧
/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x0d
 *  bit13~0，    接收到的有效字节数目
*/
uint16_t spi_rx_sta = 0;
uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
uint8_t spi_rx_buffer[SPI_RXBUFFERSIZE];
//uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* HAL库使用的串口接收缓冲 */

instructment1_dictionary_1 euip1_dict_1[16];  //设备1字典
instructment1_dictionary_2 euip1_dict_2[16];
instructment2_dictionary_1 euip2_dict_1[48];  //设备2字典
instructment2_dictionary_2 euip2_dict_2[48];
datainquire_dictionary_1 datasave_eqip_1;     //数据存储器字典
datainquire_dictionary_2 datasave_eqip_2;   
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
//	HAL_SPI_Receive_IT(&hspi1, (uint8_t *)spi_rx_buffer, SPI_RXBUFFERSIZE);
	__HAL_SPI_ENABLE(&hspi1); /* 使能SPI1 */
//	spi1_read_write_byte(0XFF); /* 启动传输, 实际上就是产生8个时钟脉冲, 达到清空DR的作用, 非必需 */
  /* USER CODE END SPI1_Init 2 */

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */
	
  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_RX Init */
    hdma_spi1_rx.Instance = DMA2_Stream0;
    hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_rx.Init.Mode = DMA_NORMAL;
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmarx,hdma_spi1_rx);

    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(spiHandle,hdmatx,hdma_spi1_tx);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

    /* SPI1 DMA DeInit */
    HAL_DMA_DeInit(spiHandle->hdmarx);
    HAL_DMA_DeInit(spiHandle->hdmatx);
  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
 * @brief       SPI1读写一个字节数据
 * @param       txdata  : 要发送的数据(1字节)
 * @retval      接收到的数据(1字节)
 */
//uint8_t spi1_read_write_byte(uint8_t txdata)
//{
//    uint8_t rxdata;
//    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, 1000);
//    return rxdata; /* 返回收到的数据 */
//}
/**
 * @brief       SPI1读写一个字节数据
 * @param       txdata  : 要发送的数据(1字节)
 * @retval      接收到的数据(1字节)
 */
uint8_t SPI1_ReadWriteByte(uint8_t txdata)
{  
     uint8_t retry=0;     
//     while((SPI1->SR&1<<1)==0)
//     {
//          retry++;
//          if(retry>200)
//					{
//						return 0;
//					}
//     }     
//     (__IO uint8_t)SPI1->DR = txdata; 
//     retry=0;
     while((SPI1->SR&1<<0)==0)  
     {
          retry++;
          if(retry>200)
					{
							return 0;
					}
     }             
     return SPI1->DR;                
}




void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
void DMA2_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

  /* USER CODE END DMA2_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

  /* USER CODE END DMA2_Stream3_IRQn 1 */
}

// 发送数据
void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size)
{
    // 发送同步特殊帧
		memset(spi_tx_rx_buffer, 0x00, sizeof(spi_tx_rx_buffer));
		spi_tx_rx_buffer[0] = SYNC_FRAME;
		
		
    // 分包发送数据
    uint32_t packets = (size + PACKET_SIZE - 1) / PACKET_SIZE; // 总的数据包数
		printf("packets %d\r\n", packets);
    for (uint32_t i = 0; i < packets; i++)
    {
        uint32_t remaining = size - i * PACKET_SIZE;
				printf("remaining %d\r\n", remaining);
				uint16_t data_crc;
			  // 如果剩余的字节不足PACKET_SIZE的话是此时发送的内容补0操作
        uint32_t packetSize = remaining > PACKET_SIZE ? PACKET_SIZE : remaining;
				spi_tx_rx_buffer[1] = (packetSize>>8)& 0xff;
				spi_tx_rx_buffer[2] = (packetSize)& 0xff;
			  memset(&spi_tx_rx_buffer[3], 0x00, sizeof(spi_tx_rx_buffer)-3); //不论发送数据是否为PACKET_SIZE的, 直接补0
				printf("packetSize %d\r\n", packetSize); // 12
//				data_crc = crc16tablefast(&data[i * PACKET_SIZE], size);  //crc校验值 只计算数据部分的校验值每128字节进行一次校验
//				memcpy(&spi_tx_rx_buffer[3],(const void*)data[i * PACKET_SIZE], PACKET_SIZE);
				memcpy(&spi_tx_rx_buffer[3],(const void*)&data[i * PACKET_SIZE], (PACKET_SIZE));
//				printf("spi_tx_rx_buffer[3] %X %X\r\n", spi_tx_rx_buffer[3],spi_tx_rx_buffer[4]);
				data_crc = crc16tablefast(spi_tx_rx_buffer, 8); // 也可以只校验前8为要传输的数据
				spi_tx_rx_buffer[PACKET_SIZE+5-2] = data_crc & 0xff;
				spi_tx_rx_buffer[PACKET_SIZE+5-1] = (data_crc>> 8) & 0xff;
				printf("data_crc %X %X\r\n", spi_tx_rx_buffer[PACKET_SIZE+5-2], spi_tx_rx_buffer[PACKET_SIZE+5-1]);
				
//        HAL_SPI_Transmit_DMA(&hspi1, &data[i * PACKET_SIZE], packetSize);
				HAL_SPI_Transmit_DMA(&hspi1, spi_tx_rx_buffer, sizeof(spi_tx_rx_buffer));
			
        // 等待DMA传输完成
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
				{
//						printf("spi1 dma send error\r\n");
				}
			

    }
}

// 接收数据
void SPI_SlaveReceiveData_DMA(uint8_t* data, uint32_t* size)
{
    // 等待同步特殊帧
    uint8_t syncFrame;
    while (syncFrame != SYNC_FRAME)
    {
        HAL_SPI_Receive(&hspi1, &syncFrame, 1, HAL_MAX_DELAY);
    }

    // 接收数据总大小
    uint8_t sizeData[4];
    HAL_SPI_Receive(&hspi1, sizeData, 4, HAL_MAX_DELAY);
    *size = (sizeData[0] << 24) | (sizeData[1] << 16) | (sizeData[2] << 8) | sizeData[3];

    // 分包接收数据
    uint32_t packets = (*size + PACKET_SIZE - 1) / PACKET_SIZE; // 总的数据包数
    for (uint32_t i = 0; i < packets; i++)
    {
        uint32_t remaining = *size - i * PACKET_SIZE;
        uint32_t packetSize = remaining > PACKET_SIZE ? PACKET_SIZE : remaining;
				
        HAL_SPI_Receive_DMA(&hspi1, &data[i * PACKET_SIZE], packetSize);

        // 等待DMA传输完成
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) 
				{
						// 其他操作
				}
    }
}

//void SPI1_IRQHandler(void)
//{
//		uint32_t timeout = 0;
//		uint32_t maxDelay = 0x1FFFF;
//		
//		HAL_SPI_IRQHandler(&hspi1);
//		timeout = 0;
//		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) /* 等待就绪 */
//		{
//				timeout++;                              /* 超时处理 */
//				if(timeout > maxDelay)
//				{
//						break;
//				}
//		}
//		timeout=0;
//		
//		/* 一次处理完成之后，重新开启中断并设置RxXferCount为1 */
//		while (HAL_SPI_Receive_IT(&hspi1, (uint8_t *)spi_rx_buffer, SPI_RXBUFFERSIZE) != HAL_OK)
//		{
//				timeout++;                              /* 超时处理 */
//				if (timeout > maxDelay)
//				{
//						break;
//				}
//		}
//	
//}



//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
//{  
////	Set_SPI_CS(1);
//	if (hspi == (&hspi1))
//	{
//			// 0x8000 = 1000 0000 0000 0000,     bit15 还未接收完毕
//			if((spi_rx_sta & 0x8000) == 0)      /* 接收未完成 */
//			{
////						\r 的十六进制表示为：0x0D 
////						\n 的十六进制表示为：0x0A
//						// 0x4000 = 0100 0000 0000 0000 bit14为接收到0x0d的标志位
//						// 此时认为已经接收到了16384位数据，按照规定下一位(16385)数据一定要是0x0a,这样才能组成0x0d 0x0a
//						// 检查是否接收到了 0x0d（回车）字符。通过位运算判断 g_usart_rx_sta 的第 14 位是否为 1
//						if(spi_rx_sta & 0x4000)         /* 接收到了0x0d */ 
//            {
//								// 判断接收到的字符是否为 0x0a（换行）字符。如果不是，则表示接收到的数据有误，需要重新开始接收
//                if(spi_rx_buffer[0] != 0x0a) 
//                {
//                    spi_rx_sta = 0;         /* 接收错误,重新开始 -这一步修改更改 */
//                }
//                else 
//                {
//                    spi_rx_sta |= 0x8000;       /* 接收完成了 将接收状态的最高位置为 1，表示接收完成*/
//                }
//            }
//						else                                /* 还没收到0X0D */
//            {
//								// 判断是否收到的数据已经是结尾了，是的话直接拼装好截止符
//							  // 如果不是的话则不能这样做，就按照从0到16383字节接收总共能接收16384个字节数量一次为一组数据
//                // 判断接收到的字符是否为 0x0d（回车）字符。如果是，则在接收状态中记录已接收到回车标志位
//							  if(spi_rx_buffer[0] == 0x0d) 
//                {
//										// 将接收状态的第 14 位设置为 1，表示已接收到回车
//                    spi_rx_sta |= 0x4000; 
//                }
//                else
//                {
//										// 3FFF = 16384 = 0011 1111 1111 1111 = 2^14
//										// 
//                    spi_recevice[spi_rx_sta & 0X3FFF] = spi_rx_buffer[0] ;
//                    spi_rx_sta++;
//                    if(spi_rx_sta > (SPI_REC_LEN - 1))
//                    {
//                        spi_rx_sta = 0;     			/* 接收数据错误,重新开始接收 */
//                    }
//                }
//            }
//			}
//	}	
//}

////写flash-给数据存储的板子
//void SPI_read_flash(void)
//{
//	uint16_t i,j,k;
//	uint8_t flash_frame_head[9] = {0x08,0x5a,0x5a,0x5a,0x5a,0x5a,0x5a,0x5a,0x5a};
//	uint8_t flash_frame_end[9] = {0x08, 0xa5,0xa5,0xa5,0xa5,0xa5,0xa5,0xa5,0xa5};
////	v_Get1302(time_ds1302);  //获取当前时刻
//	// flash-usb就是带数据存储芯片的板子
//	//1.向flash-usb板发送帧头
//	if (spi_rx_flag == 1)
//	{
//		  spi_rx_flag = 0;
//	}
//	/**
//	接收首帧8字节长度数据
//	8字节时间
//	26256字节的CAN设备1的字典数据
//		16*(1 + 256*5 + 72*5) 16个1640字节的数据为一个完整的帧
//	19248字节的CAN设备2的字典数据
//		48*(1 + 40*5 + 40*5)  48个401字节的数据为一个帧
//	401字节的数据存储的字典数据
//		1 + 40*5 + 40*5				1个401字节的数据为一个帧
//	8字节尾帧数据
//	*/
//	for(i=0;i<8;i++)
//	{
//		SPI1_ReadWriteByte(flash_frame_head[i]);
//	}
//	//2.向flash-usb板发送时间
//	for(i=0;i<8;i++)
//	{
//		SPI1_ReadWriteByte(time_ds1302[i]);
//	}
//	//3.向flash-usb板发送can设备1的字典
//	for(i=0;i<16;i++)
//	{
//		SPI1_ReadWriteByte(i+0x10);
//		k = 0;
//		for(j=0;j<256;j++)
//		{
//			SPI1_ReadWriteByte(euip1_dict_1[i].data_index);
//			SPI1_ReadWriteByte((euip1_dict_1[i].data_index)>>8);
//			SPI1_ReadWriteByte(euip1_dict_1[i].sub_index[j]);
//			SPI1_ReadWriteByte(euip1_dict_1[i].data_information[k]);
//			SPI1_ReadWriteByte(euip1_dict_1[i].data_information[k+1]);
//			k += 2;
//		}
//		k = 0;
//		for(j=0;j<72;j++)
//		{
//			SPI1_ReadWriteByte(euip1_dict_2[i].data_index);
//			SPI1_ReadWriteByte((euip1_dict_2[i].data_index)>>8);
//			SPI1_ReadWriteByte(euip1_dict_2[i].sub_index[j]);
//			SPI1_ReadWriteByte(euip1_dict_2[i].data_information[k]);
//			SPI1_ReadWriteByte(euip1_dict_2[i].data_information[k+1]);
//			k += 2;
//		}
//	}
//	//4.向flash-usb板发送can设备2的字典
//	for(i=0;i<48;i++)
//	{
//		SPI1_ReadWriteByte(i+0x20);
//		k = 0;
//		for(j=0;j<40;j++)
//		{
//			SPI1_ReadWriteByte(euip2_dict_1[i].data_index);
//			SPI1_ReadWriteByte((euip2_dict_1[i].data_index)>>8);
//			SPI1_ReadWriteByte(euip2_dict_1[i].sub_index[j]);
//			SPI1_ReadWriteByte(euip2_dict_1[i].data_information[k]);
//			SPI1_ReadWriteByte(euip2_dict_1[i].data_information[k+1]);
//			k += 2;
//		}
//		k = 0;
//		for(j=0;j<40;j++)
//		{
//			SPI1_ReadWriteByte(euip2_dict_2[i].data_index);
//			SPI1_ReadWriteByte((euip2_dict_2[i].data_index)>>8);
//			SPI1_ReadWriteByte(euip2_dict_2[i].sub_index[j]);
//			SPI1_ReadWriteByte(euip2_dict_2[i].data_information[k]);
//			SPI1_ReadWriteByte(euip2_dict_2[i].data_information[k+1]);
//			k += 2;
//		}
//	}
//	//5.向flash-usb板发送存储设备的字典
//	for(i=0;i<1;i++)
//	{
//		SPI1_ReadWriteByte(i+0x00);
//		k = 0;
//		for(j=0;j<40;j++)
//		{
//			SPI1_ReadWriteByte(datasave_eqip_1.data_index);
//			SPI1_ReadWriteByte((datasave_eqip_1.data_index)>>8);
//			SPI1_ReadWriteByte(datasave_eqip_1.sub_index[j]);
//			SPI1_ReadWriteByte(datasave_eqip_1.data_information[k]);
//			SPI1_ReadWriteByte(datasave_eqip_1.data_information[k+1]);
//			k += 2;
//		}
//		k = 0;
//		for(j=0;j<40;j++)
//		{
//			SPI1_ReadWriteByte(datasave_eqip_2.data_index);
//			SPI1_ReadWriteByte((datasave_eqip_2.data_index)>>8);
//			SPI1_ReadWriteByte(datasave_eqip_2.sub_index[j]);
//			SPI1_ReadWriteByte(datasave_eqip_2.data_information[k]);
//			SPI1_ReadWriteByte(datasave_eqip_2.data_information[k+1]);
//			k += 2;
//		}
//	}
//	//6.向flash-usb板发送帧尾
//	for(i=0;i<8;i++)
//	{
//		SPI1_ReadWriteByte(flash_frame_end[i]);
//	}
//}




/* USER CODE END 1 */

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

#define DELAY_BETWEEN_PACKETS 100   // 每锟斤拷锟斤拷锟捷帮拷之锟斤拷锟斤拷锟绞笔憋拷洌?锟斤拷位锟斤拷锟斤拷锟诫）	
uint16_t spi_rx_sta = 0;
//uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
uint8_t spi_rx_buffer[SPI_RXBUFFERSIZE];
spi_send *spi_send_buf;
//uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* HAL库使用的串口接收缓冲 */

instructment1_dictionary_1 euip1_dict_1[16];  //设备1字典
instructment1_dictionary_2 euip1_dict_2[16];
//instructment2_dictionary_1 euip2_dict_1[48];  //设备2字典
//instructment2_dictionary_2 euip2_dict_2[48];
//datainquire_dictionary_1 datasave_eqip_1;     //数据存储器字典
//datainquire_dictionary_2 datasave_eqip_2;   
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


spi_send *spi_send_buf;

/**尽可能的让接收端简化步骤，只用验证接收的数据是否正确，其他的尽可能的不做数据处理
 * 封包操作
 * data: 全部的数据？还是固定长度的数据呢？：暂时先考虑已经处理过的数据
 * size：跟随上个参数引起的歧义，是那个长度？：长度暂时不变-定长发送数据
 * count 当前传输的数据总量，比如我传输的是euip1_dict_2[i],这个数据总量就是216=72+144，euip1_dict_1[i]字典，此时的数据总量就是256+512
*/
void data_packaging(uint8_t* data, uint32_t size, uint16_t count)
{
		
    uint16_t packetSize=0;
    uint16_t data_crc=0;
    spi_send_buf->head[0] = 0xA0;
    spi_send_buf->head[1] = 0xB0;
		
		// index = E1
    spi_send_buf->index[0] = (data[0]>>8)&0xFF;
    spi_send_buf->index[1] = (data[0]&0xFF);

    //如果帧包数          据部分满足256字字节，此时传输数据256个字节
    
    if (size > PACKET_SIZE) 
    {
      packetSize = PACKET_SIZE;
    }
    else
    {
      // 如果传输不足256字节，此时只复制数据段中要保存的数据字节，其余不足的空补领操作。同时要说明数据传输的字节个数
      // 例如传输的数据为40+80 此时就有120字节的数量，就有大概一半的传输数量不足，此时packetSize就等于120，
      // 尽可能的一个帧包保存一个设备节点的数据 
			packetSize = size;
    } 
		printf("packetSize %d, count %d\r\n", packetSize, count);
		
		// 当前帧传输的有效字节数 packetSize
    spi_send_buf->data_len[0] = (packetSize>>8)&0xFF;
    spi_send_buf->data_len[1] = (packetSize) & 0xFF;

		// 一个数据的总长度 
	  spi_send_buf->all_data_len[0] = (count>>8)&0xFF;// 计算要发送的数据的长度
    spi_send_buf->all_data_len[1] = (count) & 0xFF;
		
		// 计算前8位字符的CRC校验值作为帧传输的依据
    data_crc = crc16tablefast(&spi_send_buf->head[0], 8);         // 也可以只校验前8为要传输的数据
    spi_send_buf->crc_data[0] = (data_crc>> 8) & 0xFF;
    spi_send_buf->crc_data[1] = data_crc & 0xFF;
		
    // 先补0，如果256个字节全部使用，这样的话 0x00 也会被覆盖掉，如果没有用到也能达到清楚上一次发送数据的作用
    memset(&spi_send_buf->info, 0x01, PACKET_SIZE); 
    memcpy(&spi_send_buf->info, &data[1], packetSize);
			// 测试代码
//		for (uint8_t i = 0; i< packetSize;i++)
//		{
//			if (data[i] != NULL)
//			{
//					printf("data[%d]=%X ", i, data[i]);
//			}
//			if (i%20 == 0)printf("\r\n");
//				
//		}
//		printf("\r\n");
//		spi_send_buf->info[0] = 0xE1;
//		printf("head %X %X\r\n", spi_send_buf->head[0], spi_send_buf->head[1]);
//		printf("index %X %X\r\n", spi_send_buf->index[0], spi_send_buf->index[1]);
//		printf("data_len %X %X\r\n", spi_send_buf->data_len[0], spi_send_buf->data_len[1]);
//		printf("all_data_len %X %X\r\n", spi_send_buf->all_data_len[0], spi_send_buf->all_data_len[1]);
//		printf("data_crc %X %X\r\n", spi_send_buf->crc_data[0], spi_send_buf->crc_data[1]);
			// 查看要传输的值 spi_send_buf->info[i]的具体内容
//		for (uint8_t i = 0; i< packetSize;i++)
//		{
//			if (spi_send_buf->info[i] != NULL)
//			{
//					printf("info[%d]=%X ", i, spi_send_buf->info[i]);
//			}
//			
//			if (i%20 == 0)printf("\r\n");
//		}
//		printf("\r\n");
//		printf("sizeof(spi_send) %d", sizeof(spi_send));

	  HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)spi_send_buf, sizeof(spi_send));
		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
		{
		}
}

# if 1
// size 要传输的字节数量，一个设备节点
void SPI_MasterSendData_DMA_2(uint8_t* data, uint16_t data_size, uint32_t one_size)
{
    // 分包发送数据
    uint8_t i, j;
		uint16_t index = 0;
		uint8_t *ptr=NULL;
//    uint32_t send_data_size = data_size;   // 1+256+512 单个设备节点的数据总量 769
    uint8_t send_count = 0;  								 // 一个设备节点需要发送多少次数据
		uint16_t one_struct_size = one_size;   //一个结构体的大小
    int remaining =0;   // 剩余待发送的字节数量
	 
		send_count = 15;		//多少个设备借节点
		printf("one_struct_size %d\r\n", one_struct_size);
    // uint32_t remaining_count = size/PACKET_SIZE; // 512+256
    uint8_t packets_count = 0; // 一个设备的的帧包的数量
    if (data_size> PACKET_SIZE)
    {
      packets_count = (data_size - 1) / PACKET_SIZE;  // 实际帧包的数量
    }
    else
    {
      packets_count = 1;                   				   // 不足一个帧包的数量，补全为一个帧包
    }
    // 16个设备，每个设备要发 x 个帧包，x=size/
		printf("packets_count %d, send_count %d, data_size %d\r\n", packets_count, send_count, data_size);
    for (i = 0; i < send_count; i++) // 16个
    {   
				ptr = &data[(one_struct_size)*i];
				index = ptr[0];
				printf("index %X, i %d\r\n",index, i);
        for(j=0; j < packets_count; j++) // 3个帧包
        {
          if (data_size < PACKET_SIZE)       // 针对不足一个PACKET_SIZE 数据包而言，每次发送的数据都是固定，
																				     // 只发一个包，其他的则是按照正常发送数据
          {
              remaining = data_size;
          }
          else
          {
              remaining = PACKET_SIZE;
          }
					printf("remaining %d\r\n", remaining);
				  data_packaging(ptr, remaining, data_size);   // data_size当前设备节点一个完整的数据的长度
        }

    }
}

#endif 
void data_unpacking(uint8_t* data, uint32_t size)
{

}




#if 0
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

#endif /* HAL_SPI_MODULE



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
//	26256字节的CAN设备1的字典数据 105.024=106个帧包才能完成发送
//		16*(1 + 256*5 + 72*5) 16个1640字节的数据为一个完整的帧
//	19248字节的CAN设备2的字典数据 77个帧包才能完成
//		48*(1 + 40*5 + 40*5)  48个401字节的数据为一个帧
//	401字节的数据存储的字典数据 2个帧包才能完成
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

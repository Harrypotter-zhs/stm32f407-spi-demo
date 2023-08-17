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


//uint8_t SYNC_FRAME = 0xAA;    // ͬ��֡
/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/
uint16_t spi_rx_sta = 0;
uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
uint8_t spi_rx_buffer[SPI_RXBUFFERSIZE];
//uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* HAL��ʹ�õĴ��ڽ��ջ��� */

instructment1_dictionary_1 euip1_dict_1[16];  //�豸1�ֵ�
instructment1_dictionary_2 euip1_dict_2[16];
instructment2_dictionary_1 euip2_dict_1[48];  //�豸2�ֵ�
instructment2_dictionary_2 euip2_dict_2[48];
datainquire_dictionary_1 datasave_eqip_1;     //���ݴ洢���ֵ�
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
	__HAL_SPI_ENABLE(&hspi1); /* ʹ��SPI1 */
//	spi1_read_write_byte(0XFF); /* ��������, ʵ���Ͼ��ǲ���8��ʱ������, �ﵽ���DR������, �Ǳ��� */
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
 * @brief       SPI1��дһ���ֽ�����
 * @param       txdata  : Ҫ���͵�����(1�ֽ�)
 * @retval      ���յ�������(1�ֽ�)
 */
//uint8_t spi1_read_write_byte(uint8_t txdata)
//{
//    uint8_t rxdata;
//    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, 1000);
//    return rxdata; /* �����յ������� */
//}
/**
 * @brief       SPI1��дһ���ֽ�����
 * @param       txdata  : Ҫ���͵�����(1�ֽ�)
 * @retval      ���յ�������(1�ֽ�)
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

// ��������
void SPI_MasterSendData_DMA(uint8_t* data, uint32_t size)
{
    // ����ͬ������֡
		memset(spi_tx_rx_buffer, 0x00, sizeof(spi_tx_rx_buffer));
		spi_tx_rx_buffer[0] = SYNC_FRAME;
		
		
    // �ְ���������
    uint32_t packets = (size + PACKET_SIZE - 1) / PACKET_SIZE; // �ܵ����ݰ���
		printf("packets %d\r\n", packets);
    for (uint32_t i = 0; i < packets; i++)
    {
        uint32_t remaining = size - i * PACKET_SIZE;
				printf("remaining %d\r\n", remaining);
				uint16_t data_crc;
			  // ���ʣ����ֽڲ���PACKET_SIZE�Ļ��Ǵ�ʱ���͵����ݲ�0����
        uint32_t packetSize = remaining > PACKET_SIZE ? PACKET_SIZE : remaining;
				spi_tx_rx_buffer[1] = (packetSize>>8)& 0xff;
				spi_tx_rx_buffer[2] = (packetSize)& 0xff;
			  memset(&spi_tx_rx_buffer[3], 0x00, sizeof(spi_tx_rx_buffer)-3); //���۷��������Ƿ�ΪPACKET_SIZE��, ֱ�Ӳ�0
				printf("packetSize %d\r\n", packetSize); // 12
//				data_crc = crc16tablefast(&data[i * PACKET_SIZE], size);  //crcУ��ֵ ֻ�������ݲ��ֵ�У��ֵÿ128�ֽڽ���һ��У��
//				memcpy(&spi_tx_rx_buffer[3],(const void*)data[i * PACKET_SIZE], PACKET_SIZE);
				memcpy(&spi_tx_rx_buffer[3],(const void*)&data[i * PACKET_SIZE], (PACKET_SIZE));
//				printf("spi_tx_rx_buffer[3] %X %X\r\n", spi_tx_rx_buffer[3],spi_tx_rx_buffer[4]);
				data_crc = crc16tablefast(spi_tx_rx_buffer, 8); // Ҳ����ֻУ��ǰ8ΪҪ���������
				spi_tx_rx_buffer[PACKET_SIZE+5-2] = data_crc & 0xff;
				spi_tx_rx_buffer[PACKET_SIZE+5-1] = (data_crc>> 8) & 0xff;
				printf("data_crc %X %X\r\n", spi_tx_rx_buffer[PACKET_SIZE+5-2], spi_tx_rx_buffer[PACKET_SIZE+5-1]);
				
//        HAL_SPI_Transmit_DMA(&hspi1, &data[i * PACKET_SIZE], packetSize);
				HAL_SPI_Transmit_DMA(&hspi1, spi_tx_rx_buffer, sizeof(spi_tx_rx_buffer));
			
        // �ȴ�DMA�������
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
				{
//						printf("spi1 dma send error\r\n");
				}
			

    }
}

// ��������
void SPI_SlaveReceiveData_DMA(uint8_t* data, uint32_t* size)
{
    // �ȴ�ͬ������֡
    uint8_t syncFrame;
    while (syncFrame != SYNC_FRAME)
    {
        HAL_SPI_Receive(&hspi1, &syncFrame, 1, HAL_MAX_DELAY);
    }

    // ���������ܴ�С
    uint8_t sizeData[4];
    HAL_SPI_Receive(&hspi1, sizeData, 4, HAL_MAX_DELAY);
    *size = (sizeData[0] << 24) | (sizeData[1] << 16) | (sizeData[2] << 8) | sizeData[3];

    // �ְ���������
    uint32_t packets = (*size + PACKET_SIZE - 1) / PACKET_SIZE; // �ܵ����ݰ���
    for (uint32_t i = 0; i < packets; i++)
    {
        uint32_t remaining = *size - i * PACKET_SIZE;
        uint32_t packetSize = remaining > PACKET_SIZE ? PACKET_SIZE : remaining;
				
        HAL_SPI_Receive_DMA(&hspi1, &data[i * PACKET_SIZE], packetSize);

        // �ȴ�DMA�������
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) 
				{
						// ��������
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
//		while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) /* �ȴ����� */
//		{
//				timeout++;                              /* ��ʱ���� */
//				if(timeout > maxDelay)
//				{
//						break;
//				}
//		}
//		timeout=0;
//		
//		/* һ�δ������֮�����¿����жϲ�����RxXferCountΪ1 */
//		while (HAL_SPI_Receive_IT(&hspi1, (uint8_t *)spi_rx_buffer, SPI_RXBUFFERSIZE) != HAL_OK)
//		{
//				timeout++;                              /* ��ʱ���� */
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
//			// 0x8000 = 1000 0000 0000 0000,     bit15 ��δ�������
//			if((spi_rx_sta & 0x8000) == 0)      /* ����δ��� */
//			{
////						\r ��ʮ�����Ʊ�ʾΪ��0x0D 
////						\n ��ʮ�����Ʊ�ʾΪ��0x0A
//						// 0x4000 = 0100 0000 0000 0000 bit14Ϊ���յ�0x0d�ı�־λ
//						// ��ʱ��Ϊ�Ѿ����յ���16384λ���ݣ����չ涨��һλ(16385)����һ��Ҫ��0x0a,�����������0x0d 0x0a
//						// ����Ƿ���յ��� 0x0d���س����ַ���ͨ��λ�����ж� g_usart_rx_sta �ĵ� 14 λ�Ƿ�Ϊ 1
//						if(spi_rx_sta & 0x4000)         /* ���յ���0x0d */ 
//            {
//								// �жϽ��յ����ַ��Ƿ�Ϊ 0x0a�����У��ַ���������ǣ����ʾ���յ�������������Ҫ���¿�ʼ����
//                if(spi_rx_buffer[0] != 0x0a) 
//                {
//                    spi_rx_sta = 0;         /* ���մ���,���¿�ʼ -��һ���޸ĸ��� */
//                }
//                else 
//                {
//                    spi_rx_sta |= 0x8000;       /* ��������� ������״̬�����λ��Ϊ 1����ʾ�������*/
//                }
//            }
//						else                                /* ��û�յ�0X0D */
//            {
//								// �ж��Ƿ��յ��������Ѿ��ǽ�β�ˣ��ǵĻ�ֱ��ƴװ�ý�ֹ��
//							  // ������ǵĻ��������������Ͱ��մ�0��16383�ֽڽ����ܹ��ܽ���16384���ֽ�����һ��Ϊһ������
//                // �жϽ��յ����ַ��Ƿ�Ϊ 0x0d���س����ַ�������ǣ����ڽ���״̬�м�¼�ѽ��յ��س���־λ
//							  if(spi_rx_buffer[0] == 0x0d) 
//                {
//										// ������״̬�ĵ� 14 λ����Ϊ 1����ʾ�ѽ��յ��س�
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
//                        spi_rx_sta = 0;     			/* �������ݴ���,���¿�ʼ���� */
//                    }
//                }
//            }
//			}
//	}	
//}

////дflash-�����ݴ洢�İ���
//void SPI_read_flash(void)
//{
//	uint16_t i,j,k;
//	uint8_t flash_frame_head[9] = {0x08,0x5a,0x5a,0x5a,0x5a,0x5a,0x5a,0x5a,0x5a};
//	uint8_t flash_frame_end[9] = {0x08, 0xa5,0xa5,0xa5,0xa5,0xa5,0xa5,0xa5,0xa5};
////	v_Get1302(time_ds1302);  //��ȡ��ǰʱ��
//	// flash-usb���Ǵ����ݴ洢оƬ�İ���
//	//1.��flash-usb�巢��֡ͷ
//	if (spi_rx_flag == 1)
//	{
//		  spi_rx_flag = 0;
//	}
//	/**
//	������֡8�ֽڳ�������
//	8�ֽ�ʱ��
//	26256�ֽڵ�CAN�豸1���ֵ�����
//		16*(1 + 256*5 + 72*5) 16��1640�ֽڵ�����Ϊһ��������֡
//	19248�ֽڵ�CAN�豸2���ֵ�����
//		48*(1 + 40*5 + 40*5)  48��401�ֽڵ�����Ϊһ��֡
//	401�ֽڵ����ݴ洢���ֵ�����
//		1 + 40*5 + 40*5				1��401�ֽڵ�����Ϊһ��֡
//	8�ֽ�β֡����
//	*/
//	for(i=0;i<8;i++)
//	{
//		SPI1_ReadWriteByte(flash_frame_head[i]);
//	}
//	//2.��flash-usb�巢��ʱ��
//	for(i=0;i<8;i++)
//	{
//		SPI1_ReadWriteByte(time_ds1302[i]);
//	}
//	//3.��flash-usb�巢��can�豸1���ֵ�
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
//	//4.��flash-usb�巢��can�豸2���ֵ�
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
//	//5.��flash-usb�巢�ʹ洢�豸���ֵ�
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
//	//6.��flash-usb�巢��֡β
//	for(i=0;i<8;i++)
//	{
//		SPI1_ReadWriteByte(flash_frame_end[i]);
//	}
//}




/* USER CODE END 1 */

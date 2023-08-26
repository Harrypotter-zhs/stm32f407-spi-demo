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
#include "crc16.h"	
#include "string.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */


//uint8_t SYNC_FRAME = 0xAA;    // ͬ��֡
/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/

#define DELAY_BETWEEN_PACKETS 100   // ÿ�����ݰ�֮�����ʱʱ��?��λ�����룩	
uint16_t spi_rx_sta = 0;
//uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
uint8_t spi_rx_buffer[PACKET_SIZE];

spi_send *spi_send_buf; // ���ͻ�����
spi_send *spi_rx_buf;   // ���ջ�����
//static uint8_t one_flag = 1;
//#define rx_one_data_len 0
static uint16_t instructment_index=0;
uint16_t rx_one_data_len = 0;        //һ���豸�ڵ�Ҫ����������ݳ���
static uint16_t Current_len_now =0;
uint8_t flag_save = 0;
                                     // ÿ���ӽڵ��������128�ֽڴ�������Ҫ����6����Ч֡��.��Ҫ���������ֽ�����
static uint8_t device_number_one = 0, device_number_two = 0,flag_open=0, flag_same=0;
//uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* HAL��ʹ�õĴ��ڽ��ջ��� */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
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
//uint8_t SPI1_ReadWriteByte(uint8_t txdata)
//{  
//     uint8_t retry=0;     
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
//     while((SPI1->SR&1<<0)==0)  
//     {
//          retry++;
//          if(retry>200)
//					{
//							return 0;
//					}
//     }             
//     return SPI1->DR;                
//}



// DMA����
//void DMA2_Stream0_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

//  /* USER CODE END DMA2_Stream0_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_spi1_rx);
//  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

//  /* USER CODE END DMA2_Stream0_IRQn 1 */
//}

/**
  * @brief This function handles DMA2 stream3 global interrupt.
  */
// DMA����
//void DMA2_Stream3_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

//  /* USER CODE END DMA2_Stream3_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_spi1_tx);
//  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

//  /* USER CODE END DMA2_Stream3_IRQn 1 */
//}


spi_send *spi_send_buf;

/**�����ܵ��ý��ն˼򻯲��裬ֻ����֤���յ������Ƿ���ȷ�������ľ����ܵĲ������ݴ���
 * �������
 * data: ȫ�������ݣ����ǹ̶����ȵ������أ�����ʱ�ȿ����Ѿ������������
 * size����ǰ֡����Ч���ݳ���
 * count ��ǰ��������������������Ҵ������euip1_dict_2[i],���������������216=72+144��euip1_dict_1[i]�ֵ䣬��ʱ��������������256+512
*/
void data_packaging(uint8_t* data, uint32_t size, uint16_t count)
{
		
    uint16_t packetSize=0;
    uint16_t data_crc=0;
    spi_send_buf->head[0] = 0xA0; // ֡ͷ��Ϊͬ��֡
//    spi_send_buf->head[1] = 0xB0;
		spi_send_buf->head[1] = 0xEB; // �豸�����豸00-01H �豸 10-1Fh, 20-4Fh��
		// index = ������ 2000 ����2100
    spi_send_buf->index[0] = (data[0]>>8)&0xFF;
    spi_send_buf->index[1] = (data[0]&0xFF);

    //���֡����          �ݲ�������256���ֽڣ���ʱ��������256���ֽ�
    
//    if (size > PACKET_SIZE) 
//    {
//      packetSize = PACKET_SIZE;
//    }
//    else
    {
      // ������䲻��256�ֽڣ���ʱֻ�������ݶ���Ҫ����������ֽڣ����಻��Ŀղ��������ͬʱҪ˵�����ݴ�����ֽڸ���
      // ���紫�������Ϊ40+80 ��ʱ����120�ֽڵ����������д��һ��Ĵ����������㣬��ʱpacketSize�͵���120��
      // �����ܵ�һ��֡������һ���豸�ڵ������ 
			packetSize = size;
    } 
		printf("packetSize %d, count %d\r\n", packetSize, count);
		
		// ��ǰ֡�������Ч�ֽ��� packetSize
    spi_send_buf->data_len[0] = (packetSize>>8)&0xFF;
    spi_send_buf->data_len[1] = (packetSize) & 0xFF;

		// һ�����ݵ��ܳ��� 
	  spi_send_buf->all_data_len[0] = (count>>8)&0xFF;// ����Ҫ���͵����ݵĳ���
    spi_send_buf->all_data_len[1] = (count) & 0xFF;
		
		// ����ǰ8λ�ַ���CRCУ��ֵ��Ϊ֡���������
    data_crc = crc16tablefast(&spi_send_buf->head[0], 8);         // Ҳ����ֻУ��ǰ8ΪҪ���������
    spi_send_buf->crc_data[0] = (data_crc>> 8) & 0xFF;
    spi_send_buf->crc_data[1] = data_crc & 0xFF;
		
    // �Ȳ�0�����256���ֽ�ȫ��ʹ�ã������Ļ� 0x00 Ҳ�ᱻ���ǵ������û���õ�Ҳ�ܴﵽ�����һ�η������ݵ�����
    memset(&spi_send_buf->info, 0x01, PACKET_SIZE); 
    memcpy(&spi_send_buf->info, &data[1], packetSize);
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)spi_send_buf, sizeof(spi_send));
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
	{
	}
}

# if 1
// size Ҫ������ֽ�������һ���豸�ڵ�
/**
data_size һ���豸�ڵ�Ҫ�������Ч���ݳ��� ����2+256+512������2+40+80
*/
void SPI_MasterSendData_DMA_2(uint8_t* data, uint16_t data_size, uint32_t one_size, uint8_t times)
{
    // �ְ���������
    uint8_t i, j;
		uint16_t index = 0;
		uint8_t *ptr=NULL;
//    uint32_t send_data_size = data_size;   // 1+256+512 �����豸�ڵ���������� 769
    uint8_t send_count = 0;  								 // һ���豸�ڵ���Ҫ���Ͷ��ٴ�����
		uint16_t one_struct_size = one_size;   //һ���ṹ��Ĵ�С
    int remaining =0;   // ʣ������͵��ֽ�����
	 
//		send_count = 48;		//���ٸ��豸��ڵ�
		send_count = times;
		printf("one_struct_size %d\r\n", one_struct_size);
    // uint32_t remaining_count = size/PACKET_SIZE; // 512+256
    uint8_t packets_count = 0; // һ���豸�ĵ�֡��������
    if (data_size> PACKET_SIZE)
    {
		packets_count = (data_size) / PACKET_SIZE;  // ʵ��֡��������
		if ((data_size%PACKET_SIZE) > 0) //��������ʣ���ֽ�δ���;����·������
				packets_count = packets_count + 1;
    }
    else
    {
          packets_count = 1;                   				   // ����һ��֡������������ȫΪһ��֡��
    }
    // 16���豸��ÿ���豸Ҫ�� x ��֡����x=size/
		printf("packets_count %d, send_count %d, data_size %d\r\n", packets_count, send_count, data_size);
    for (i = 0; i < send_count; i++) // 16��
    {   
		ptr = &data[(one_struct_size)*i];
		index = ptr[0];
		printf("index %X, i %d\r\n",index, i);
        for(j=0; j < packets_count; j++) // 3��֡��
        {
          if (data_size < PACKET_SIZE)       // ��Բ���һ��PACKET_SIZE ���ݰ����ԣ�ÿ�η��͵����ݶ��ǹ̶���
																				     // ֻ��һ���������������ǰ���������������
          {	
														
				remaining = data_size;
								          
          }
          else
          {
				if (j == (packets_count -1))
				{
					if ((data_size%PACKET_SIZE) == 0)
					{
						remaining = PACKET_SIZE;
					}
					else
					{
							remaining = (data_size%PACKET_SIZE);
					}
				}
				else
				{
						remaining = PACKET_SIZE;
				}
          }
			
//					memcpy(&spi_send_buf->info, &data[1], packetSize); // ����仰ʹ�õ�
//					printf("remaining %d\r\n", remaining);
			data_packaging(ptr, remaining, data_size);   // data_size��ǰ�豸�ڵ�һ�����������ݵĳ���
        }

    }
}

#endif 
// Ŀ����ֻ�ý��գ�����ֻ�����CRCУ��ֵ�ж��Ƿ���ȷ�������ȷ�Ļ����Ǿ������������ݣ������ж�

// 0=1B 1=A0 2=B0 3=0 4=12 5=0 6=D8 7=0 8=D8 9=DA 10=C3 11=12 12=1B 13=1B 14=1B 15=1B 16=1B 17=1B 18=1B 19=1B 
// ��Ҫдһ�����±�־���δ�ɹ��������ʾΪ 0 �ɹ����ʾΪ 1 
// size û�õ�

void data_copy_device(uint8_t device,uint16_t index, uint8_t* data, uint32_t size)
{

	if((device == 0x00)) //��ȡ�������ݴ洢��A����ֵ�
	{
			if (index == 0x2000)
			{
				datasave_eqip_1.data_index = index;
				memcpy(&datasave_eqip_1.sub_index, data, size);
			}
			else if(index == 0x2100)
			{
				datasave_eqip_2.data_index = index;
				memcpy(&datasave_eqip_1.sub_index, data, size);
			}
	}
	else if((device == 0x01))
	{
		// ���û�����޷��鿴����
			
	}
	else if((device >= 0x10) && (device <= 0x1F))
	{
		if (index == 0x2000)
		{
			euip1_dict_1[(device-16)].data_index = index;
			// memcpy(&spi_send_buf->info, &data[1], packetSize);
			memcpy(&euip1_dict_1[(device-16)].sub_index, data, size);
		}
		else if(index == 0x2100)
		{
			euip1_dict_2[(device-10)].data_index = index;
			memcpy(&euip1_dict_2[(device-16)].sub_index, data, size);
			
//			printf("euip1_dict_2[%d].index%X\r\n", (device-16), ((euip1_dict_2[(device-16)].data_index>>8)&0xFF));
			printf("euip1_dict_2[%d].sub_index%X\r\n", (device-16),euip1_dict_2[(device-16)].sub_index[20]);
		}
	}
	else if((device >= 0x20) && (device <= 0x4F))
	{
		if (index == 0x2000)
		{
			euip2_dict_1[(device-32)].data_index = index;
			// memcpy(&spi_send_buf->info, &data[1], packetSize);
			memcpy(&euip2_dict_1[(device-32)].sub_index, data, size);
		}
		else if(index == 0x2100)
		{
			euip2_dict_2[(device-32)].data_index = index;
			memcpy(&euip2_dict_2[(device-32)].sub_index, data, size);
		}
	}
	else
	{
		;
	}



}

void data_unpacking(uint8_t* data, uint32_t size)
{
		uint8_t rx_data_crc[2];
		uint16_t send_data_crc;
	    uint16_t rx_len=0; 
		uint8_t* ptr = data;
		// static uint8_t rx_times = 0;
		uint8_t rx_num = 0;                  // ������Ҫ���յ�֡�Ĵ���������euip1_dict_1[16] ��16���豸�ӽڵ㣬
		
		// if (one_flag == 1 && (*ptr != NULL))
		if ((*ptr != NULL))
		{
			  // �鿴֡ͷ�Ƿ���ȷ�������ȷ ������ݴ��� ����֡ͷ�Ƿ���ȷ
				if ((ptr[0] == 0xA0) && (ptr[1] == 0xB0))
				{
					{
						rx_data_crc[0] = ptr[8];
						rx_data_crc[1] = ptr[9];
					}
					device_number_one = ptr[2]; // �豸�ӽڵ�
					// ����˳��ͽ��յ�˳����һ���ģ�Ҫ���̶������ݾ���
					send_data_crc = crc16tablefast(ptr,8);
					//�Ա�CRCУ��ֵ�Ƿ���ȷ��������д������ݿ�ʼ
					//�豸У����ȷ-˵��������ȷ-У�鷽ʽ���Ǻܶ�
					if (send_data_crc == ((rx_data_crc[0]<<8)|rx_data_crc[1])) 
					{		
						printf("received ok\r\n");
						// ÿ���豸�Ľڵ�� 2000 ����2100 
						instructment_index = (((uint16_t)ptr[3])<<8);
						// ��ǰ�豸��x������֡�е���Ч���ݳ���
						rx_len = (ptr[4]<<8|(ptr[5]));
						// ��ǰ�豸��x���ܵ���Ч���ݳ���
						Current_len_now = Current_len_now + rx_len;
						rx_one_data_len = (ptr[6]<<8|(ptr[7])); 
						
						// ���Բ鿴���յ������Ƿ���ȷ�������ȡ���Ե� 
						// 10H-2100-216�ֽڣ���128 88 ���η���
						printf("index %X\r\n", instructment_index);
						printf("device %X\r\n", device_number_one);
						// printf("data_len %X\r\n", rx_len);
						// printf("data_all_len %X\r\n", rx_one_data_len);
						printf("data_len %d\r\n", rx_len);
						printf("data_all_len %d\r\n", rx_one_data_len);
						printf("Current_len_now %d\r\n", Current_len_now);
						// ��ÿ���豸Ϊ�ڵ������һ���豸������x��֡
						// ÿ�ν�����Ҫ������ܵ�֡��
						if ((rx_one_data_len % PACKET_SIZE) == 0) // ǡ������ȫ����
						{
							rx_num = rx_one_data_len / PACKET_SIZE; // 768/128 = 6,�����
						}
						else																					// ���ܱ���ȫ������������
						{
							if ((rx_one_data_len / PACKET_SIZE) != 0) // ����218%128 = 90,218/128 = 1 
							{
									rx_num = (rx_one_data_len / PACKET_SIZE) + 1; // 1+1 ��Ҫ��������֡������ȫ����������
							}
						}
						printf("rx_num %d\r\n", rx_num);
						if (rx_num == 1) // ֻ��һ������֡���������е����ݴ���,�Ǿͱ�ʾ�������֡�Ϳ��Ա��渳ֵ����
						{
							// ���ݸ�ֵ����
//									    memset(&spi_send_buf->info, 0x01, PACKET_SIZE); 
//											memcpy(&spi_send_buf->info, &data[1], packetSize);
							
							// memset(&spi_rx_buf->info, 0x00, PACKET_SIZE);	
							// // ����һ֡�е���Ч����rx_len
							// memcpy(&spi_rx_buf->info, &ptr[10], rx_len); 			
								data_copy_device(device_number_one, instructment_index, ptr, rx_len);
						}
						else if(rx_num > 1) //���֡���䣬����Ϊ2��֡����
						{	
							if(flag_open == 0) // ���֡���䣬��ʱ��ʾΪ��һ�ν��ո��豸������֡
							{
									device_number_one = ptr[2];//��ǰ�豸�����ݽ��յ��ĵ�һ֡
									flag_same = 1;  // 
									flag_open = 1;  // Ϊ��һ֡��׼��
									printf("device_number_one %X\r\n", device_number_one);
							}
							else if((flag_open == 1))// ��ǰ�豸�ӽڵ�û������һ֡��������һ���豸�ӽڵ�ģ����ҽ��ո����ݵĵ�2 3 ����֡����
							{	
								device_number_two = ptr[2];
								if (device_number_two == device_number_one)		
								{
										flag_same = flag_same + 1;
										printf("device number same\r\n");
								}
							}
							if (flag_same != 0) // ���ն�֡����������ֽڱ��� ��һ֡һ�ڽ�������֡��
							{
								printf("data processing\r\n");
								printf("flag_same %d\r\n", flag_same);
								data_copy_device(device_number_one, instructment_index, ptr, rx_len);
								// �������-��ʱ����豸������ȫ���������
								if(flag_same == rx_num)
								{
									flag_open = 0;
									flag_same = 0;
									Current_len_now = 0;
									printf("receiver all data one device OK %X\r\n", device_number_two);
								}
									// // ���ݸ�ֵ����
									// memset(&spi_rx_buf->info, 0x00, PACKET_SIZE);	
									// // ����һ֡�е���Ч����rx_len
									// memcpy(&spi_rx_buf->info, &ptr[10], rx_len); 
							}
						}
				}
				else // CRCУ�����
				{
					printf("decvice %X %X crc error ", ptr[2], ptr[3]);
					printf("send_data_crc %X\r\n", send_data_crc);
				}
			}
			else // �������ݴ���-����˵��û�н��յ���ͷ�����ݣ�������֡������
			{
					printf("head error %X %X\r\n", ptr[0], ptr[1]);
			}
		}
		else if(ptr == NULL)
		{
			printf("spi receive error\r\n");
		}
		// one_flag = 0;
}

//void SPI_MasterReceiveData_DMA_2(uint8_t* data, uint16_t data_size, uint32_t one_size, uint8_t times)
//void SPI_MasterReceiveData_DMA_2(void)
//{
//		if (HAL_DMA_PollForTransfer(&hdma_spi1_rx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY) == HAL_OK)
//    {
//      /* DMA transfer completed, data is available in RxBuffer */
//      // ������յ�������
//			one_flag = 1;
//			printf("SPI_MasterReceiveData_DMA_2\r\n");
//    }
//}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    // ���ݽ���ȫ�����ʱ�Ĵ���
//    // ...
//		one_flag = 1;
//		HAL_SPI_Receive_DMA(&hspi1, spi_rx_buffer, 256);
//		printf("HAL_SPI_RxCpltCallback");
////		data_unpacking((uint8_t*)spi_receive_buf, 0);
//}

#if 0
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

#endif /* HAL_SPI_MODULE


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









/* USER CODE END 1 */

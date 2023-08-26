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


//uint8_t SYNC_FRAME = 0xAA;    // 同步帧
/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x0d
 *  bit13~0，    接收到的有效字节数目
*/

#define DELAY_BETWEEN_PACKETS 100   // 每锟斤拷锟斤拷锟捷帮拷之锟斤拷锟斤拷锟绞笔憋拷洌?锟斤拷位锟斤拷锟斤拷锟诫）	
uint16_t spi_rx_sta = 0;
//uint8_t spi_tx_rx_buffer[PACKET_SIZE+5];
uint8_t spi_rx_buffer[PACKET_SIZE];

spi_send *spi_send_buf; // 发送缓存区
spi_send *spi_rx_buf;   // 接收缓存区
//static uint8_t one_flag = 1;
//#define rx_one_data_len 0
static uint16_t instructment_index=0;
uint16_t rx_one_data_len = 0;        //一个设备节点要传输的总数据长度
static uint16_t Current_len_now =0;
uint8_t flag_save = 0;
                                     // 每个子节点如果按照128字节传输则需要接收6个有效帧数.需要计算出这个字节数量
static uint8_t device_number_one = 0, device_number_two = 0,flag_open=0, flag_same=0;
//uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* HAL库使用的串口接收缓冲 */

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



// DMA接收
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
// DMA发送
//void DMA2_Stream3_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream3_IRQn 0 */

//  /* USER CODE END DMA2_Stream3_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_spi1_tx);
//  /* USER CODE BEGIN DMA2_Stream3_IRQn 1 */

//  /* USER CODE END DMA2_Stream3_IRQn 1 */
//}


spi_send *spi_send_buf;

/**尽可能的让接收端简化步骤，只用验证接收的数据是否正确，其他的尽可能的不做数据处理
 * 封包操作
 * data: 全部的数据？还是固定长度的数据呢？：暂时先考虑已经处理过的数据
 * size：当前帧的有效数据长度
 * count 当前传输的数据总量，比如我传输的是euip1_dict_2[i],这个数据总量就是216=72+144，euip1_dict_1[i]字典，此时的数据总量就是256+512
*/
void data_packaging(uint8_t* data, uint32_t size, uint16_t count)
{
		
    uint16_t packetSize=0;
    uint16_t data_crc=0;
    spi_send_buf->head[0] = 0xA0; // 帧头作为同步帧
//    spi_send_buf->head[1] = 0xB0;
		spi_send_buf->head[1] = 0xEB; // 设备号码设备00-01H 设备 10-1Fh, 20-4Fh的
		// index = 索引号 2000 或者2100
    spi_send_buf->index[0] = (data[0]>>8)&0xFF;
    spi_send_buf->index[1] = (data[0]&0xFF);

    //如果帧包数          据部分满足256字字节，此时传输数据256个字节
    
//    if (size > PACKET_SIZE) 
//    {
//      packetSize = PACKET_SIZE;
//    }
//    else
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
	HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)spi_send_buf, sizeof(spi_send));
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
	{
	}
}

# if 1
// size 要传输的字节数量，一个设备节点
/**
data_size 一个设备节点要传输的有效数据长度 比如2+256+512，或者2+40+80
*/
void SPI_MasterSendData_DMA_2(uint8_t* data, uint16_t data_size, uint32_t one_size, uint8_t times)
{
    // 分包发送数据
    uint8_t i, j;
		uint16_t index = 0;
		uint8_t *ptr=NULL;
//    uint32_t send_data_size = data_size;   // 1+256+512 单个设备节点的数据总量 769
    uint8_t send_count = 0;  								 // 一个设备节点需要发送多少次数据
		uint16_t one_struct_size = one_size;   //一个结构体的大小
    int remaining =0;   // 剩余待发送的字节数量
	 
//		send_count = 48;		//多少个设备借节点
		send_count = times;
		printf("one_struct_size %d\r\n", one_struct_size);
    // uint32_t remaining_count = size/PACKET_SIZE; // 512+256
    uint8_t packets_count = 0; // 一个设备的的帧包的数量
    if (data_size> PACKET_SIZE)
    {
		packets_count = (data_size) / PACKET_SIZE;  // 实际帧包的数量
		if ((data_size%PACKET_SIZE) > 0) //但凡还有剩余字节未传送就重新封包传输
				packets_count = packets_count + 1;
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
			
//					memcpy(&spi_send_buf->info, &data[1], packetSize); // 给这句话使用的
//					printf("remaining %d\r\n", remaining);
			data_packaging(ptr, remaining, data_size);   // data_size当前设备节点一个完整的数据的长度
        }

    }
}

#endif 
// 目的是只用接收，并且只计算个CRC校验值判断是否正确，如果正确的话，那就正常接收数据，并且判断

// 0=1B 1=A0 2=B0 3=0 4=12 5=0 6=D8 7=0 8=D8 9=DA 10=C3 11=12 12=1B 13=1B 14=1B 15=1B 16=1B 17=1B 18=1B 19=1B 
// 需要写一个更新标志如果未成功更新则表示为 0 成功则表示为 1 
// size 没用到

void data_copy_device(uint8_t device,uint16_t index, uint8_t* data, uint32_t size)
{

	if((device == 0x00)) //读取的是数据存储器A板的字典
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
		// 这个没定义无法查看数据
			
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
		uint8_t rx_num = 0;                  // 计算需要接收的帧的次数，比如euip1_dict_1[16] 有16个设备子节点，
		
		// if (one_flag == 1 && (*ptr != NULL))
		if ((*ptr != NULL))
		{
			  // 查看帧头是否正确如果不正确 这个数据错误 检验帧头是否正确
				if ((ptr[0] == 0xA0) && (ptr[1] == 0xB0))
				{
					{
						rx_data_crc[0] = ptr[8];
						rx_data_crc[1] = ptr[9];
					}
					device_number_one = ptr[2]; // 设备子节点
					// 发的顺序和接收的顺序是一样的，要看固定的数据就行
					send_data_crc = crc16tablefast(ptr,8);
					//对比CRC校验值是否正确，是则进行处理数据开始
					//设备校验正确-说明数据正确-校验方式不是很对
					if (send_data_crc == ((rx_data_crc[0]<<8)|rx_data_crc[1])) 
					{		
						printf("received ok\r\n");
						// 每个设备的节点号 2000 或者2100 
						instructment_index = (((uint16_t)ptr[3])<<8);
						// 当前设备的x的数据帧中的有效数据长度
						rx_len = (ptr[4]<<8|(ptr[5]));
						// 当前设备的x的总的有效数据长度
						Current_len_now = Current_len_now + rx_len;
						rx_one_data_len = (ptr[6]<<8|(ptr[7])); 
						
						// 测试查看接收的数据是否正确，检验截取测试的 
						// 10H-2100-216字节，分128 88 两次发送
						printf("index %X\r\n", instructment_index);
						printf("device %X\r\n", device_number_one);
						// printf("data_len %X\r\n", rx_len);
						// printf("data_all_len %X\r\n", rx_one_data_len);
						printf("data_len %d\r\n", rx_len);
						printf("data_all_len %d\r\n", rx_one_data_len);
						printf("Current_len_now %d\r\n", Current_len_now);
						// 以每个设备为节点基础，一个设备可以有x个帧
						// 每次接收需要计算的总的帧数
						if ((rx_one_data_len % PACKET_SIZE) == 0) // 恰好能完全整除
						{
							rx_num = rx_one_data_len / PACKET_SIZE; // 768/128 = 6,如果是
						}
						else																					// 不能被完全整除的数据量
						{
							if ((rx_one_data_len / PACKET_SIZE) != 0) // 比如218%128 = 90,218/128 = 1 
							{
									rx_num = (rx_one_data_len / PACKET_SIZE) + 1; // 1+1 需要两个数据帧才能完全接收完数据
							}
						}
						printf("rx_num %d\r\n", rx_num);
						if (rx_num == 1) // 只用一个数据帧便可完成所有的数据传输,那就表示接收完此帧就可以保存赋值即可
						{
							// 数据赋值操作
//									    memset(&spi_send_buf->info, 0x01, PACKET_SIZE); 
//											memcpy(&spi_send_buf->info, &data[1], packetSize);
							
							// memset(&spi_rx_buf->info, 0x00, PACKET_SIZE);	
							// // 接收一帧中的有效数据rx_len
							// memcpy(&spi_rx_buf->info, &ptr[10], rx_len); 			
								data_copy_device(device_number_one, instructment_index, ptr, rx_len);
						}
						else if(rx_num > 1) //多个帧传输，假设为2个帧传输
						{	
							if(flag_open == 0) // 多个帧传输，此时表示为第一次接收该设备的数据帧
							{
									device_number_one = ptr[2];//当前设备的数据接收到的第一帧
									flag_same = 1;  // 
									flag_open = 1;  // 为下一帧做准备
									printf("device_number_one %X\r\n", device_number_one);
							}
							else if((flag_open == 1))// 当前设备子节点没错，和上一帧的数据是一个设备子节点的，并且接收该数据的第2 3 。。帧数据
							{	
								device_number_two = ptr[2];
								if (device_number_two == device_number_one)		
								{
										flag_same = flag_same + 1;
										printf("device number same\r\n");
								}
							}
							if (flag_same != 0) // 接收多帧传输的数据字节比如 第一帧一节接下来的帧数
							{
								printf("data processing\r\n");
								printf("flag_same %d\r\n", flag_same);
								data_copy_device(device_number_one, instructment_index, ptr, rx_len);
								// 接收完成-此时这个设备的数据全部接收完毕
								if(flag_same == rx_num)
								{
									flag_open = 0;
									flag_same = 0;
									Current_len_now = 0;
									printf("receiver all data one device OK %X\r\n", device_number_two);
								}
									// // 数据赋值操作
									// memset(&spi_rx_buf->info, 0x00, PACKET_SIZE);	
									// // 接收一帧中的有效数据rx_len
									// memcpy(&spi_rx_buf->info, &ptr[10], rx_len); 
							}
						}
				}
				else // CRC校验错误
				{
					printf("decvice %X %X crc error ", ptr[2], ptr[3]);
					printf("send_data_crc %X\r\n", send_data_crc);
				}
			}
			else // 接收数据错误-或者说并没有接收到开头的数据，抛弃此帧的数据
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
//      // 处理接收到的数据
//			one_flag = 1;
//			printf("SPI_MasterReceiveData_DMA_2\r\n");
//    }
//}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    // 数据接收全部完成时的处理
//    // ...
//		one_flag = 1;
//		HAL_SPI_Receive_DMA(&hspi1, spi_rx_buffer, 256);
//		printf("HAL_SPI_RxCpltCallback");
////		data_unpacking((uint8_t*)spi_receive_buf, 0);
//}

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









/* USER CODE END 1 */

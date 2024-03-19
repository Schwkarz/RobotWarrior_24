/*用于将数据发送到电脑进行显示，在RM c板上标识的是uart2，实际上使用的是f4芯片的uart1*/

#ifndef UART1_H
#define UART1_H
#include "main.h"

void UART1_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendData(uint8_t* data, uint16_t length);
void ROS_Receive_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void ROS_Receive_restart(uint16_t dma_buf_num);


#endif

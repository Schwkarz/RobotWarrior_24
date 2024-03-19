#include "stm32f4xx.h"
#include "ROS_Receive.h"
#include "uart1.h"
#include "stdio.h"
#include "INS_Task.h"
#include "camera_trigger.h"
#include "led.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include  "string.h"

//ROS出错数据上限
#define ROS_Receive_ERROR_VALUE 500

static uint8_t ROS_rx_buf[2][ROS_RX_BUF_NUM];
static ROS_Msg_t ROS_Msg;

//将串口接收到的数据转化为ROS实际的数据
void UART_to_ROS_Msg(uint8_t *uart_buf, ROS_Msg_t *ros_msg);

static void Send_Gimbal_Angle(float yaw, float pitch, uint8_t c);
const volatile fp32 *local_imu_angle;

void imuSendTask(void *pvParameters)
{
    vTaskDelay(3000);//延时等待陀螺仪初始化完毕
    {// 初始化相机pwm硬触发
        //相机外部硬触发初始化
        camera_trigger_Init();
        Camera_trigger_set(20);//20ms触发
    }
    Camera_trigger_start();

    local_imu_angle = get_INS_angle_point();
    TickType_t IMU_LastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&IMU_LastWakeTime, 20);
        
        // Send_Gimbal_Angle(local_imu_angle[0], local_imu_angle[1], 0);
    }
}

void TIM8_UP_TIM13_IRQHandler(void) 
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update) != RESET) 
    {
        Send_Gimbal_Angle(local_imu_angle[0], local_imu_angle[1], 0);
    }
    TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
}

//初始化DMA，串口1
void ROS_Init(void)
{
    ROS_Receive_Init(ROS_rx_buf[0], ROS_rx_buf[1], ROS_RX_BUF_NUM);
}

//串口中断
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART1);

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream5) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream5, DISABLE);
            this_time_rx_len = ROS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream5);
            DMA_SetCurrDataCounter(DMA2_Stream5, ROS_RX_BUF_NUM);
            DMA2_Stream5->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA2_Stream5, ENABLE);
            if(this_time_rx_len == ROS_FRAME_LENGTH)
            {
                //处理ROS数据
				UART_to_ROS_Msg(ROS_rx_buf[0], &ROS_Msg);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream5, DISABLE);
            this_time_rx_len = ROS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream5); 
            DMA_SetCurrDataCounter(DMA2_Stream5, ROS_RX_BUF_NUM);
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA2_Stream5, ENABLE);
            if(this_time_rx_len == ROS_FRAME_LENGTH)
            {
                //处理ROS数据
				UART_to_ROS_Msg(ROS_rx_buf[1], &ROS_Msg);
            }
        }
    }
}

/**
 * @brief	计算 CRC 校验码
 * @param	_pBuf	待计算的数组指针
 * @param	_usLen	待计算的数据长度
 * @return 	void
 */
void getModbusCRC16(unsigned char *_pBuf, unsigned short int _usLen)
{
    unsigned short int CRCValue = 0xFFFF;
    unsigned char i,j;

    for(i=0;i<_usLen;++i)
    {
        CRCValue  ^= *(_pBuf+i);
        for(j=0;j<8;++j)
        {
            if((CRCValue & 0x01) == 0x01)
            {
                 CRCValue = (CRCValue >> 1)^0xA001;
            }
            else 
            {
                CRCValue >>= 1;
            }           
        }
    }
    *(_pBuf + _usLen) = (CRCValue & 0xFF00) >> 8; 		// CRC 校验码高位
    *(_pBuf + _usLen + 1) = CRCValue & 0x00FF;			// CRC 校验码低位
    return;            
} 

void UART_to_ROS_Msg(uint8_t *uart_buf, ROS_Msg_t *ros_msg)
{
	if(uart_buf == NULL || ros_msg == NULL || uart_buf[0] != ROS_START_BYTE)
	{
		return;
	}

	unsigned char sum = 0x00;
	
	for(unsigned short i=0;i<uart_buf[2] - 1;i++)
		sum += uart_buf[i];
	
	if(sum == uart_buf[uart_buf[2]-1])
    {
        uint8_t* yawDataPtr = &uart_buf[3];
        uint8_t* pitchDataPtr = &uart_buf[7];
        uint8_t* depthDataPtr = &uart_buf[11];
        
        fp32 set_yaw, set_pitch, depth;
        memcpy(&set_yaw, yawDataPtr, sizeof(fp32));
        memcpy(&set_pitch, pitchDataPtr, sizeof(fp32));
        memcpy(&depth, depthDataPtr, sizeof(fp32));

        ROS_Msg.shoot_yaw = set_yaw;
        ROS_Msg.shoot_pitch = set_pitch;
        ROS_Msg.shoot_depth = depth;
    }
}


//返回ROS数据，通过指针传递方式传递信息
const ROS_Msg_t *get_ROS_Msg_point(void)
{
    return &ROS_Msg;
}

static void Send_Gimbal_Angle(float yaw, float pitch, uint8_t color)
{
    uint8_t sendBuff[13];

    sendBuff[0] = 0x42; // 设置帧头
    sendBuff[1] = 0x21; // 设置地址
    sendBuff[2] = 13;   // 设置帧长

    float data_array[] = {yaw, pitch};
    
    for (int j = 0; j < 2; j++) 
    {
        uint8_t* float_data_ptr = (uint8_t*)&data_array[j];
        for (int i = 0; i < sizeof(float); i++) 
        {
            sendBuff[3 + j * sizeof(float) + i] = float_data_ptr[i];
        }
    }
    sendBuff[11] = color;

    uint8_t check = 0x00;
    for(int i = 0; i < 12; i++)
        check += sendBuff[i];
    sendBuff[12] = check;

    Serial_SendData(sendBuff, 13);
}


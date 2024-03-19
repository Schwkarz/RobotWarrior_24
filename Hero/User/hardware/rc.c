#include "rc.h"
#include "stm32f4xx.h"

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
        /* -------------- Enable Module Clock Source ----------------------------*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3); //PC11  usart3_rx

        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        USART_DeInit(USART3);

        USART_InitStructure.USART_BaudRate = 100000;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART3, &USART_InitStructure);

        USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

        // USART_ClearFlag(USART3, USART_FLAG_RXNE);
        // USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        USART_ClearFlag(USART3, USART_FLAG_IDLE);
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

        USART_Cmd(USART3, ENABLE);

        /* ------------------ Configure USART3 NVIC -----------------------------------*/
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RC_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        //DMA1 stream1 ch4
        /* ----------------------- Configure DMA -----------------------------------*/
        DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(DMA1_Stream1);

        DMA_InitStructure.DMA_Channel = DMA_Channel_4;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART3->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = dma_buf_num;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA1_Stream1, &DMA_InitStructure);

        DMA_DoubleBufferModeConfig(DMA1_Stream1, (uint32_t)rx2_buf, DMA_Memory_0);//双缓冲模式
        DMA_DoubleBufferModeCmd(DMA1_Stream1, ENABLE);
        DMA_Cmd(DMA1_Stream1, ENABLE);
}

void RC_unable(void)
{
        USART_Cmd(USART3, DISABLE);
}

void RC_restart(uint16_t dma_buf_num)
{
        USART_Cmd(USART3, DISABLE);
        DMA_Cmd(DMA1_Stream1, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Stream1, dma_buf_num);

        USART_ClearFlag(USART3, USART_FLAG_IDLE);

        DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
        DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
        DMA_Cmd(DMA1_Stream1, ENABLE);
        USART_Cmd(USART3, ENABLE);
}

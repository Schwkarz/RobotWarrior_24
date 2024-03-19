#include "spi.h"
#include "stm32f4xx.h"


/**SPI1 GPIO Configuration    
PB4     ------> SPI1_MISO
PB3     ------> SPI1_SCK
PA7     ------> SPI1_MOSI
*/
void SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//双线双向全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//同步时钟空闲状态
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//片选nss信号由软件决定
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);
}

void SPI1_SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize)
{
    SPI1->CR1 &= 0xF7C7;
    SPI1->CR1 |= Speed;
    SPI1->CR1 |= DataSize;
    SPI1->CR1 |= 1 << 6;
}

void SPI1_DMA_Init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{

    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SPI1_RX_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(DMA2_Stream2);

    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE)
    {
        ;
    }
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = rx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = num;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);

    DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);

    DMA_Cmd(DMA2_Stream2, DISABLE); //Add a disable

    DMA_DeInit(DMA2_Stream3);

    while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE)
    {
        ;
    }
    DMA_InitStructure.DMA_Channel = DMA_Channel_3;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (SPI1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = tx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = num;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream3, &DMA_InitStructure);

    DMA_Cmd(DMA2_Stream3, DISABLE); //Add a disable
}

void SPI1_DMA_Enable(uint16_t ndtr)
{
    DMA_Cmd(DMA2_Stream3, DISABLE);
    DMA_Cmd(DMA2_Stream2, DISABLE);
    while (DMA_GetCmdStatus(DMA2_Stream3) != DISABLE)
    {
        ;
    }
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE)
    {
        ;
    }

    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
    DMA_ClearFlag(DMA2_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);

    DMA_SetCurrDataCounter(DMA2_Stream2, ndtr);
    DMA_SetCurrDataCounter(DMA2_Stream3, ndtr);
    DMA_Cmd(DMA2_Stream2, ENABLE);
    DMA_Cmd(DMA2_Stream3, ENABLE);
}


uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t retry=0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //等待发送区空
    {
        retry++;
        if(retry>200) return 0;
    }
    SPI_I2S_SendData(SPI1, TxData); //通过外设 SPIx 发送一个数据

    retry=0;
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) //等待接收完一个 byte
    { 
        retry++;
        if(retry>200) return 0;
    }
    return SPI_I2S_ReceiveData(SPI1); //返回通过 SPIx 最近接收的数据
}


uint8_t SPI1_RW_Byte(uint8_t TxData)
{
	uint8_t retry=0;
	while((SPI1->SR & SPI_SR_TXE) == 0)		//等待发送区空			  
	{
		retry++;
		if(retry>=200) return 0;
	}
    SPI1->DR = TxData;
	// *(uint8_t*)&(SPI1->DR) = TxData;

	retry = 0;
	while ((SPI1->SR & SPI_SR_RXNE) == 0) 		//等待接收完一个byte  
	{
		retry++;
		if(retry>=200) return 0;
	}
	return SPI1->DR;
    // return *(uint8_t*)&(SPI1->DR);
}

#ifndef SPI_H
#define SPI_H
#include "main.h"

void SPI1_Init(void);
void SPI1_SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize);

uint8_t SPI1_ReadWriteByte(uint8_t TxData);
uint8_t SPI1_RW_Byte(uint8_t TxData);

#endif

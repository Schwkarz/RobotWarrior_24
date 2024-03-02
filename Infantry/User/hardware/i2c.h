#ifndef I2C_H
#define I2C_H
#include "main.h"

void I2C_Software_Init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
uint8_t I2C_WaitAck(void);//0 正确应答；1 无应答
void I2C_SendByte(uint8_t Byte);
uint8_t I2C_ReadByte(void);

void I2C_Single_Read(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t* REG_data);
void I2C_Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);


#endif

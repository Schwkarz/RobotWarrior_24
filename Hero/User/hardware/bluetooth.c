#include "bluetooth.h"
#include <stdarg.h>
#include "uart1.h"

/**
 * @brief   通过蓝牙向手机发送数据，支持 int、float 数据
 * @param   string  将要发送的所有数据写入
 * @return  void
 */
void Bluetooth_Send(char *string, ...)
{
    Serial_SendByte(0XA5);                      //发送数据包头
    char *ptr_string = string;                  //避免直接使用string，导致string指向的地址发生变化 
    char *byte_data;
    short *short_data;
    int int_data;
    float float_data;
    void* void_ptr = &float_data;
    uint8_t* float_buff = (uint8_t*)void_ptr;
    va_list ap;                                 //定义一个va_list类型变量ap
    va_start(ap, string);                       //初始化va，使va指向string的下一个地址 
    uint8_t sum = 0x00;                         //定义和校验数据位
    uint8_t sum_buff = 0;                       //处理32位转换为8位
    while(*ptr_string != '\0')
    {
        if(*ptr_string == '%')                  //格式控制符 
        {
            switch(*++ptr_string)
            {
                case 'c':
                    byte_data = va_arg(ap,char*);
                    sum_buff = *byte_data;
                    sum += sum_buff;
                    Serial_SendByte(sum_buff);
                    break;
                case 'h':
                    if(*++ptr_string == 'd')
                    {
                        short_data = va_arg(ap,short*);
                        for (int i = 0; i < 2; i++)
                        {
                            sum_buff = (uint16_t)*short_data & (0xFF << 8*i);
                            sum += sum_buff;
                            Serial_SendByte(sum_buff);
                        }
                    }
                    break;
                case 'd':
                    int_data = va_arg(ap, int);  //检索变量
                    for (int i = 0; i < 4; i++)
                    {
                        sum_buff = (uint32_t)int_data & (0xFF << 8*i);
                        sum += sum_buff;
                        Serial_SendByte(sum_buff);
                    }
                    break;
                case 'f':
                    float_data = va_arg(ap, double);
                    for (int i = 0; i < 4; i++)
                    {
                        sum_buff = float_buff[i];
                        sum += sum_buff;
                        Serial_SendByte(sum_buff);
                    }
                default:
                    break;
            }
        }
        ptr_string++;  //指向下一个地址 
    } 
    Serial_SendByte(sum);       //发送和校验
    Serial_SendByte(0x5A);      //发送数据包尾
    va_end(ap);                 //关闭ap
}

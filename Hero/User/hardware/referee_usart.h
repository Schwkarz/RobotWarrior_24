#ifndef REFEREE_USART_H
#define REFEREE_USART_H
#include "main.h"

void referee_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void referee_restart(uint16_t dma_buf_num);

#endif

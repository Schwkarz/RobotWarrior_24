#include "stm32f4xx.h"
#include "relays.h"
#include "remote_control.h"

const RC_ctrl_t *local_rc;

void Relays_On(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_6);
}

void Relays_Off(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_6);
}

void Relays_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	Relays_On();

	local_rc = get_remote_control_point();
}

void Relays_Judge(void)
{
	if(local_rc->key.v & KEY_PRESSED_OFFSET_Z)
	{
		Relays_On();
	}
	if(local_rc->key.v & KEY_PRESSED_OFFSET_G)
	{
		Relays_Off();
	}
}

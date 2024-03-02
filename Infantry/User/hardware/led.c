#include "led.h"
#include "stm32f4xx.h"

void led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOH, &GPIO_InitStructure);

	// led_blue_on();
	led_green_on();
	// led_red_on();
}

void led_blue_on(void)
{
	GPIO_SetBits(GPIOH, GPIO_Pin_10);
}

void led_blue_off(void)
{
	GPIO_ResetBits(GPIOH, GPIO_Pin_10);
}

void led_blue_toggle(void)
{
	GPIO_ToggleBits(GPIOH, GPIO_Pin_10);
}

void led_green_on(void)
{
	GPIO_SetBits(GPIOH, GPIO_Pin_11);
}

void led_green_off(void)
{
	GPIO_ResetBits(GPIOH, GPIO_Pin_11);
}

void led_green_toggle(void)
{
	GPIO_ToggleBits(GPIOH, GPIO_Pin_11);
}

void led_red_on(void)
{
	GPIO_SetBits(GPIOH, GPIO_Pin_12);
}

void led_red_off(void)
{
	GPIO_ResetBits(GPIOH, GPIO_Pin_12);
}

void led_red_toggle(void)
{
	GPIO_ToggleBits(GPIOH, GPIO_Pin_12);
}

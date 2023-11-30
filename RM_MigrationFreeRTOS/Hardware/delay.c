#include "stm32f4xx.h"
#include "delay.h"
#include "FreeRTOS.h"

static uint16_t fac_us;
static uint16_t fac_ms;
void delay_init(uint8_t SYSCLK)
{
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //选择外部时钟  HCLK/8
    fac_us=SYSCLK/8;             //硬件分频,fac_us得出的就是每个us需要的systick时钟数   
    fac_ms=(u16)fac_us*1000;     //代表每个ms需要的systick时钟数,在ucos下,代表每个ms需要的时钟数
} 

void delay_us(uint32_t nus)
{       
    u32 temp;     
    SysTick->LOAD=nus*fac_us; //时间加载 (SYSCLK/8/1000000) * nus
    SysTick->VAL=0x00;        //清空计数器
    SysTick->CTRL=0x01 ;      //开始倒数  
    do
    {
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16))); //等待时间到达   
    SysTick->CTRL=0x00;       //关闭计数器
    SysTick->VAL =0X00;       //清空计数器
}

void delay_ms(uint16_t nms)
{       
    u32 temp;     
    SysTick->LOAD=(u32)nms*fac_ms; //时间加载 (SYSCLK/8/1000)*nms
    SysTick->VAL =0x00;           //清空计数器
    SysTick->CTRL=0x01 ;          //开始倒数 
    do
    {
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16))); //等待时间到达   
    SysTick->CTRL=0x00;       //关闭计数器
    SysTick->VAL =0X00;       //清空计数器
}

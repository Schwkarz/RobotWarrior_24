#include "stm32f4xx.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"
#include "led.h"
#include "uart1.h"
#include "stdio.h"

#define START_TASK_PRIO 1
#define START_STAKE_SIZE 128
static TaskHandle_t StartTask_Handler;

void StartTask(void *parameter)
{
	//进入临界段
	taskENTER_CRITICAL();
	
	//删除开始任务
	vTaskDelete(StartTask_Handler);
	//退出临界段
	taskEXIT_CRITICAL();

}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	led_Init();
	delay_init((uint8_t)180);
	UART1_Init();
	
	xTaskCreate((TaskFunction_t)StartTask,
				(const char*)"start_task",
				(uint16_t)START_STAKE_SIZE,
				(void *)NULL,
				(UBaseType_t)START_TASK_PRIO,
				(TaskHandle_t*)&StartTask_Handler);

	vTaskStartScheduler(); 

	while(1)
	{
		//正常启动调度器后，不会运行到此处
	}
}


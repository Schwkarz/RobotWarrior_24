#include "RM_Cilent_UI.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "ui.h"
#include "ui_interface.h"



// void UI_Task(void *parameter)
// {
//     Graph_Data Graph2;

//     // uint8_t ui_arr[] = {0xA5,0x15,0x00,0x00,0xBC,0x01,0x03,0x01,0x01,0x65,0x00,0x65,0x01,0x6E,0x69,0x6C,0x01,0x00,0x00,0x00,0x01,0x00,0x8F,0x43,0x00,0x00,0x0F,0x19,0x4F,0x13};

//     uint8_t ui_arr[] = {0xA5,0x15,0x00,0x45,0xC5,0x01,0x03,0x01,0x01,0x65,0x00,0x65,0x01,0x6E,0x69,0x6C,0x41,0x02,0x00,0x00,0x01,0x00,0x8F,0x43,0x00,0x00,0x0F,0x19,0x39,0x6D};
//     while (1)
//     {
//         memset(&Graph2, 0, sizeof(Graph2));

//         // // 画圆和线

//         Line_Draw(&Graph2, "line_center", UI_Graph_ADD, 9, UI_Color_Main, 1, 960, 540, 960, 200);
//         taskENTER_CRITICAL();
//             UI_ReFresh(1, Graph2);
//         taskEXIT_CRITICAL();
//         vTaskDelay(2000);


//         // int len = sizeof(ui_arr) / sizeof(uint8_t);
//         // UART6_Send_Data(ui_arr, len);
//         // vTaskDelay(200);
//     }
// }


void UI_Task(void *parameter)
{
    ui_init_default_Ungroup();

    while(1)
    {
        ui_update_default_Ungroup();
        
        vTaskDelay(200);
    }
}
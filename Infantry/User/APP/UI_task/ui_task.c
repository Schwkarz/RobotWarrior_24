#include "RM_Cilent_UI.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"

void UI_Task(void *parameter)
{
    //Graph1为指针,Graph2为结构体
    Graph_Data* Graph1;
    Graph_Data Graph2;
    while(1)
    {
        //内存设置为0
        memset(Graph1, 0, sizeof(Graph1));
        memset(&Graph2, 0, sizeof(Graph2));

        //画圆和线
        Circle_Draw(Graph1,"circle_center",UI_Graph_ADD,0,UI_Color_Main,3,960,540,100);
        Line_Draw(&Graph2,"line_center",UI_Graph_ADD,0,UI_Color_Main,1,960,540,960,200);
        
        UI_ReFresh(2,*Graph1,Graph2);
        vTaskDelay(100);
    }
}
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ui.h"

#include "gimbal_behaviour.h"
#include "chassis_behaviour.h"

void ui_update_judge();


const Gimbal_Control_t* local_gimbal_control_t;
gimbal_behaviour_e local_gimbal_behaviour;
chassis_behaviour_e local_chassis_behaviour;

void UI_Task(void *parameters)
{
    // strcpy(ui_default_group1_Text_1->string, "WUT");
    
    ui_init_default_group1();

    strcpy(ui_default_group1_Text_1->string, "WUT");

    while(1)
    {
        ui_update_judge();

        // ui_init_default_group1();

        ui_update_default_group1();

        vTaskDelay(100);
    }
}


void ui_update_judge()
{
    local_gimbal_behaviour = get_gimbal_behaviour();
    local_chassis_behaviour = get_chasis_behaviour();

    if(local_gimbal_behaviour == GIMBAL_AUTO_SHOOT)
    {
        strcpy(ui_default_group1_Text_2->string, "AUTOAIM:ON");
    }
    else
    {
        strcpy(ui_default_group1_Text_2->string, "AUTOAIM:OFF");
    }

    if(local_chassis_behaviour == CHASSIS_ROTATION)
    {
        strcpy(ui_default_group1_Text_3->string, "ROTATE:ON");
    }
    else
    {
        strcpy(ui_default_group1_Text_3->string, "ROTATE:OFF");
    }
}

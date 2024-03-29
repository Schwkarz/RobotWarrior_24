// 写给下一届学弟：我是画ui的lwt，英雄的ui我懒得改了，主要是标定的线改起来有点麻烦，而且动态的判断加起来很麻烦，交给你们了！！（可以参照步兵的代码）
// 过意不去，还是加了

#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ui.h"
#include "ui_interface.h"

#include "gimbal_behaviour.h"
#include "chassis_behaviour.h"
#include "chassis_remote_control.h"
#include "remote_control.h"
#include "referee.h"
#include "shoot.h"

void ui_update_judge();

#define SHOOT_READY_COLOR 2
#define SHOOT_READY_FRIC_COLOR 5
#define SHOOT_STOP_COLOR 3
#define GIMBAL_AUTOSHOOT_COLOR 2
#define GIMBAL_NOAUTOSHOOT_COLOR 3
#define CHASSIS_ROTATION_COLOR 2
#define CHASSIS_NOROTATION_COLOR 3

typedef struct
{
    uint8_t shoot_last_color;
    uint8_t autoaim_last_color;
    uint8_t rotation_last_color;
} last_color_t;

const Gimbal_Control_t *local_gimbal_control_t;
const chassis_move_t *local_chassis_control_t;
const RC_ctrl_t *local_rc_t;
const shoot_control_t *loacl_shoot_t;
chassis_behaviour_e local_chassis_mode;
gimbal_behaviour_e local_gimbal_mode;

last_color_t ui_last_color;

void UI_Task(void *parameters)
{
    vTaskDelay(UI_DELAY_TIME);

    for (int i = 0; i < 100; i++)
    {
        ui_init_default_group1();
        ui_init_default_group0();
    }

    local_rc_t = get_remote_control_point();
    loacl_shoot_t = get_shoot_control_point();
    local_gimbal_control_t = get_gimbal_control_point();
    local_chassis_control_t = get_chassis_control_point();

    {
        ui_last_color.autoaim_last_color = 3;
        ui_last_color.shoot_last_color = 3;
        ui_last_color.rotation_last_color = 3;
    }

    while (1)
    {
        local_chassis_mode = get_chasis_behaviour();

        if (local_rc_t->key.v == UI_INIT_KEYBOARD)
        {
            ui_init_default_group1();
            ui_init_default_group0();
        }

        ui_update_judge();

        vTaskDelay(100);
    }
}


void ui_update_judge()
{
    uint8_t ui_change = 0;

    if (loacl_shoot_t->shoot_mode == SHOOT_READY)
    {
        ui_default_group0_shoot_flag->color = SHOOT_READY_COLOR;
        if (ui_last_color.shoot_last_color != SHOOT_READY_COLOR)
        {
            ui_change++;
        }
        ui_last_color.shoot_last_color = ui_default_group0_shoot_flag->color;
    }
    else if (loacl_shoot_t->shoot_mode == SHOOT_READY_FRIC)
    {
        ui_default_group0_shoot_flag->color = SHOOT_READY_FRIC_COLOR;
        if (ui_last_color.shoot_last_color != SHOOT_READY_FRIC_COLOR)
        {
            ui_change++;
        }
        ui_last_color.shoot_last_color = ui_default_group0_shoot_flag->color;
    }
    else
    {
        ui_default_group0_shoot_flag->color = SHOOT_STOP_COLOR;
        if (ui_last_color.shoot_last_color != SHOOT_STOP_COLOR)
        {
            ui_change++;
        }
        ui_last_color.shoot_last_color = ui_default_group0_shoot_flag->color;
    }

    if (local_gimbal_control_t->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_AUTO_SHOOT || local_gimbal_mode == GIMBAL_AUTO_SHOOT)
    {
        ui_default_group0_autoaim_flag->color = GIMBAL_AUTOSHOOT_COLOR;
        if (ui_last_color.autoaim_last_color != GIMBAL_AUTOSHOOT_COLOR)
        {
            ui_change++;
        }
        ui_last_color.autoaim_last_color = ui_default_group0_autoaim_flag->color;
    }
    else
    {
        ui_default_group0_autoaim_flag->color = GIMBAL_NOAUTOSHOOT_COLOR;
        if (ui_last_color.autoaim_last_color != GIMBAL_AUTOSHOOT_COLOR)
        {
            ui_change++;
        }
        ui_last_color.autoaim_last_color = ui_default_group0_autoaim_flag->color;
    }

    if (local_chassis_control_t->chassis_mode == CHASSIS_ROTATION || local_chassis_mode == CHASSIS_ROTATION)
    {
        ui_default_group0_rotation_flag->color = CHASSIS_ROTATION_COLOR;
        if(ui_last_color.rotation_last_color != CHASSIS_ROTATION_COLOR)
        {
            ui_change++;
        }
        ui_last_color.rotation_last_color = ui_default_group0_rotation_flag->color;
    }
    else
    {
        ui_default_group0_rotation_flag->color = CHASSIS_NOROTATION_COLOR;
        if(ui_last_color.rotation_last_color != CHASSIS_NOROTATION_COLOR)
        {
            ui_change++;
        }
        ui_last_color.rotation_last_color = ui_default_group0_rotation_flag->color;
    }


    if(ui_change)
    {
        ui_update_default_group0();
        ui_update_default_group1();
    }
}

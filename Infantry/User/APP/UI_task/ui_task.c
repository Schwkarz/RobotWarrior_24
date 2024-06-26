/**
  ****************************RM Warrior 2023****************************
  * @file       ui_task.c/h
  * @brief      启动任务，将一个个任务开启，分配资源，给定任务优先级,
  * @note       学弟/学妹（？）你好！我是21级画ui的lwt，我们战队的UI基于南航开源https://ui.bismarck.xyz/
                进行ui绘制后，请更改接口：
                1. #define SEND_MESSAGE(message, length) print(message,length) 将print更换为我们自己uart6发送的函数
                2. nuaa_ui/ui_types.h中加入#define MESSAGE_PACKED __attribute__((packed))
                3. 更改#define UI_SELF_ID get_robot_id()，这样即可自动获取id
                4. 建议将动态需要刷新的ui放入同一个group，每次只刷新这个group，否则可能不会刷新

                local_chassis_control_t的指针读取底盘状态可能不会成功，请使用local_chassis_mode
                local_gimbal_control_t的指针读取底盘状态可能不会成功，请使用local_gimbal_mode
                这两个问题按理来说不应该，有待学弟你debug了，但现在也不是不能用

  * @warning    目前存在的问题：
                1. 上电初始化时发送不一定能够和裁判系统通信，导致比赛开始时ui显示不出来，我设置了一个按键"r"来重新init，但是
                目前来看，init的ui是一点一点出来的，最后打桂信科的那把我的字符串一直没初始化出来，原因猜测是被其他中断打断导致
                串口的发送有问题，这个现象在RTOS中比较常见，可以加入临界段解决，但是没有测试过，，
                2. 目前切换自瞄AUTOAIM时，不能正常的将autoaim_flag变为绿色，原因不详，需测试。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/4/1         pxx              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************


*/

#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

#include "ui.h"
#include "ui_interface.h"

#include "gimbal_behaviour.h"
#include "chassis_behaviour.h"
#include "chassis_task.h"
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

    ui_update_default_group1();
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
    //只有需要更改时，才发送updata
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

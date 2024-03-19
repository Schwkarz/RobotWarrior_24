/**
  ****************************RM Warrior 2023****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/2/         pxx              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       4
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00545324260857041989782339471524f       // 2PI/60/(3591/187)
#define MOTOR_ECD_TO_ANGLE          0.000019970370880995190055505595881022f   // PI / (8192*3591/187)
#define FULL_COUNT                  1975                                      // 3591/2
//拨弹速度+
#define TRIGGER_SPEED               15.0f
#define CONTINUE_TRIGGER_SPEED      10.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0       //子弹已到达限位开关
#define SWITCH_TRIGGER_OFF          1       //子弹未到达限位开关

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f    //判断为卡弹的最低速度
#define BLOCK_TIME                  500     //判断为卡弹的时长限制
#define REVERSE_TIME                200
#define REVERSE_SPEED_SET           -1.5f    //设置倒转速度
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f
#define PI_FIFTY                    0.0628f
#define PI_SIX                      0.47759877559829887307710723054658f


//拨弹轮电机PID
#define TRIGGER_SPEED_PID_KP        3800.0f
#define TRIGGER_SPEED_PID_KI        10.0f
#define TRIGGER_SPEED_PID_KD        3.0f

#define TRIGGER_BULLET_PID_MAX_OUT  12000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 2000.0f

#define TRIGGER_READY_PID_MAX_OUT   16000.0f
#define TRIGGER_READY_PID_MAX_IOUT  2000.0f



#define SHOOT_HEAT_REMAIN_VALUE     80

typedef enum
{
    SHOOT_STOP = 0,         //停止射击，摩擦轮停止转动
    SHOOT_READY_FRIC,       //摩擦轮启动，直到达到指定转速，软件自动进入SHOOT_READY_BULLET
    SHOOT_READY_BULLET,
    SHOOT_READY,            //射击准备就绪
    SHOOT_BULLET,           //射击
    SHOOT_CONTINUE_BULLET,  //持续射击
    SHOOT_DONE,
} shoot_mode_e;

typedef enum
{
    power_on_init = 0,      //上电拨弹轮准备初始化
    Enter_the_init_state,   //进入初始化状态，进行子弹上膛
    Finish_the_init_state,  //完成初始化，子弹已经上膛
    Is_stuck_Bullet,        //判断为已经卡弹，或者子弹已经上膛
    Bullet_Alraedy,         //倒转完成，子弹已经就位，可以准备发射了
    Start_Shoot_bullet,     //开始射击
} power_on_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    PidTypeDef trigger_motor_pid;
    PidTypeDef trigger_anger_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int16_t ecd_count;//电机轴转动圈数

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    power_on_mode_e move_flag;

    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
const shoot_control_t *get_shoot_control_point(void);

#endif

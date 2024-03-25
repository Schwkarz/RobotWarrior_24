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

#include "shoot.h"
#include "main.h"
#include "led.h"

#include "arm_math.h"
#include "user_lib.h"
#include "trigger.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "pid.h"
// #include "detect_task.h"

#include "rc_handoff.h"
#include "buzzer.h"
#include "Laser.h"

#define shoot_fric1_on(pwm) fric1_on((pwm))
#define shoot_fric2_on(pwm) fric2_on((pwm))
#define shoot_fric_off()    fric_off()     

#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

static void shoot_limit_pwm_set(void);

shoot_control_t shoot_control;          //射击数据

/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //电机指针
    shoot_control.shoot_motor_measure = get_Trigger_Motor_Measure_Point();
    //初始化PID
    PID_Init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    //更新数据
    shoot_feedback_update();
    ramp_init(&shoot_control.fric1_ramp, SHOOT_CONTROL_TIME, FRIC_DOWN, FRIC_OFF);
    ramp_init(&shoot_control.fric2_ramp, SHOOT_CONTROL_TIME, FRIC_DOWN, FRIC_OFF);
    shoot_control.fric_pwm1 = FRIC_OFF;
    shoot_control.fric_pwm2 = FRIC_OFF;
    shoot_control.ecd_count = 0;
    shoot_control.angle = shoot_control.shoot_motor_measure->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
}

/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{

    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据


    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
        // // Laser_Off();
        // buzzer_off();
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //设置拨弹轮的速度
        shoot_control.speed_set = 0.0f;
        Laser_On();
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        {
            shoot_control.trigger_speed_set = 0.0f;
            shoot_control.speed_set = 0.0f;
        }
        shoot_control.trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        buzzer_off();
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //设置拨弹轮的拨动速度,并开启堵转反转处理
        if(shoot_control.high_speed_on)
        {
            shoot_control.trigger_speed_set = HIGH_TRIGGER_SPEED;
        }
        else
        {
            shoot_control.trigger_speed_set = LOW_TRIGGER_SPEED;
        }
        trigger_motor_turn_back();
    }

    if(shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.given_current = 0;
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, -SHOOT_FRIC_PWM_ADD_VALUE);
    }
    else
    {
        // shoot_laser_on(); //激光开启
        //计算拨弹轮电机PID
        PID_Calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
        shoot_control.given_current = (int16_t)(shoot_control.trigger_motor_pid.out);
        if(shoot_control.shoot_mode < SHOOT_READY)
        {
            shoot_control.given_current = 0;
        }
        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&shoot_control.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        ramp_calc(&shoot_control.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
    }

    shoot_control.fric_pwm1 = (uint16_t)(shoot_control.fric1_ramp.out);
    shoot_control.fric_pwm2 = (uint16_t)(shoot_control.fric2_ramp.out);
    shoot_fric1_on(shoot_control.fric_pwm1);
    shoot_fric2_on(shoot_control.fric_pwm2);


    return shoot_control.given_current;
}

/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static bool_t last_shoot_switch = 0;
    static bool_t last_fric_switch = 0;

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_fric_on(shoot_control.shoot_rc->rc.ch[SHOOT_RC_MODE_CHANNEL]) && last_fric_switch == 0 && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_fric_on(shoot_control.shoot_rc->rc.ch[SHOOT_RC_MODE_CHANNEL]) && last_fric_switch == 0 && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //可以使用键盘开启摩擦轮
    if ((shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //可以使用键盘关闭摩擦轮
    else if ((shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    // static uint32_t Tcount = 0;
    // if(get_shoot_power_status())
    // {
    //     Tcount++;
    //     if(Tcount >= 5000)
    //     {
    //         //空操作
    //         // led_green_on();
    //     }
    //     else
    //     {
    //         // led_green_off();
    //         shoot_control.shoot_mode = SHOOT_STOP;
    //     }
    // }
    // else
    // {
    //     Tcount = 0;
    //     // led_green_off();
    //     shoot_control.shoot_mode = SHOOT_STOP;
    // }

    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && shoot_control.fric1_ramp.out == shoot_control.fric1_ramp.max_value && shoot_control.fric2_ramp.out == shoot_control.fric2_ramp.max_value)
    {
        shoot_control.shoot_mode = SHOOT_READY;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_shoot(shoot_control.shoot_rc->rc.ch[4]) && last_shoot_switch == 0) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
        }
    }


    if(shoot_control.shoot_mode > SHOOT_READY_FRIC)
    {
        //鼠标长按或者开关长期处于下档一直进入射击状态 保持连发
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY;
        }

        if(shoot_control.press_l_time == PRESS_LONG_TIME)
        {
            shoot_control.high_speed_on = 1;
        }
        else
        {
            shoot_control.high_speed_on = 0;
        }
    }

    //枪口热量限制
    get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    if((shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {
        if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            shoot_control.shoot_mode = SHOOT_READY;
        }
    }

    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_shoot_switch = switch_is_shoot(shoot_control.shoot_rc->rc.ch[4]);
	last_fric_switch = switch_is_fric_on(shoot_control.shoot_rc->rc.ch[4]);
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{

    // static fp32 speed_fliter_1 = 0.0f;
    // static fp32 speed_fliter_2 = 0.0f;
    // static fp32 speed_fliter_3 = 0.0f;

    // //拨弹轮电机速度滤波一下
    // static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    // //二阶低通滤波
    // speed_fliter_1 = speed_fliter_2;
    // speed_fliter_2 = speed_fliter_3;
    // speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    // shoot_control.speed = speed_fliter_3;

    shoot_control.speed = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

#if TRIGGER_TURN
    shoot_control.speed = -shoot_control.speed;
#endif

    // //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    // if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    // {
    //     shoot_control.ecd_count--;
    // }
    // else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    // {
    //     shoot_control.ecd_count++;
    // }

    //编码器丢帧，使用软件记上一次的编码值
    static int16_t last_ecd = 0;
    if (shoot_control.shoot_motor_measure->ecd - last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }
    last_ecd = shoot_control.shoot_motor_measure->ecd;


    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;

    //微动开关
    // shoot_control.key = BUTTEN_TRIG_PIN;
     
    //鼠标按键
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;
    //长按计时
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_shoot(shoot_control.shoot_rc->rc.ch[4]))
    {

        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    // //鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
    // static uint16_t up_time = 0;
    // if (shoot_control.press_r)
    // {
    //     up_time = UP_ADD_TIME;
    // }

    // if (up_time > 0)
    // {
    //     shoot_control.fric1_ramp.max_value = FRIC_UP;
    //     shoot_control.fric2_ramp.max_value = FRIC_UP;
    //     up_time--;
    // }
    // else
    // {
    //     shoot_control.fric1_ramp.max_value = FRIC_DOWN;
    //     shoot_control.fric2_ramp.max_value = FRIC_DOWN;
    // }
    shoot_limit_pwm_set();


}

static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = shoot_control.trigger_speed_set;
    }
    else
    {
        shoot_control.speed_set = -16;
        // shoot_control.speed_set = 0;
    }

    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        // 方式
        // shoot_control.block_time++;
        // shoot_control.reverse_time = 0;
        if(shoot_control.reverse_time > 0)
            shoot_control.reverse_time--;
        else
            shoot_control.block_time++;
        buzzer_off();
    }
    else if (shoot_control.block_time >= BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
        buzzer_on(64,20);
    }
    else
    {
        shoot_control.block_time = 0;
        buzzer_off();
    }
}

/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
    static uint8_t trigger_count = 0;//拨弹轮 间断发射计数器
    //每次拨动 1/4PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI_FOUR);
        shoot_control.move_flag = 1;
    }

    //到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        trigger_count++;
        shoot_control.trigger_speed_set = LOW_TRIGGER_SPEED;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.shoot_mode = SHOOT_READY;
        shoot_control.move_flag = 0;
        trigger_count = 0;
    }

    if(trigger_count >= 200)
    {
        shoot_control.shoot_mode = SHOOT_READY;
        shoot_control.move_flag = 0;
        trigger_count = 0;
    }
}


const shoot_control_t *get_shoot_control_point(void)
{
    return &shoot_control;
}

static void shoot_limit_pwm_set(void)
{
    uint8_t speed = get_shoot_17mm_speed_limit();
    switch (speed)
    {
    case 15:
        shoot_control.fric1_ramp.max_value = 1600;
        break;
    case 18:
        shoot_control.fric1_ramp.max_value = 1690;
        break;
    case 30:
        shoot_control.fric1_ramp.max_value = 2000;
        break;
    default:
        shoot_control.fric1_ramp.max_value = 1900;
        break;
    }
}

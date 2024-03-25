/**
  ****************************RM Warrior 2023****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
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

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"
#include "ROS_Receive.h"
#include "user_lib.h"

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 2400.0f
#define PITCH_SPEED_PID_KI 8.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 3000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
//20 0.01 0.6
#define PITCH_GYRO_ABSOLUTE_PID_KP 30.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 1.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 30.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.008f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 1.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
//4200 10 0
#define YAW_SPEED_PID_KP 3000.0f
#define YAW_SPEED_PID_KI 4.0f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
// 18
#define YAW_GYRO_ABSOLUTE_PID_KP 45.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.5f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.05f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 12.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 1.2f


//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 12.0f    //2024.3.2之前是8
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//选择用户自定义模式状态 开关通道号
#define SUPER_MODE_CHANNEL 1
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 0
#define PitchChannel 1
#define ModeChannel 0
// //掉头180 按键
// #define TurnKeyBoard KEY_PRESSED_OFFSET_F
// //掉头云台速度
// #define TurnSpeed 0.04f
//测试按键
#define TestKeyBoard KEY_PRESSED_OFFSET_G
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.000005f
// #define Pitch_RC_SEN -0.00079327f //0.005  -0.000006
#define Pitch_RC_SEN -0.0000050f
//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen 0.00005f
#define Pitch_Mouse_Sen 0.00015f
//云台编码器控制时候使用的比例
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f
//云台控制周期
#define GIMBAL_CONTROL_TIME_MS 1
#define GIMBAL_CONTROL_TIME 0.001f

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否反装
#define PITCH_TURN 0
#define YAW_TURN 1
#define TRIGGER_TURN 1

//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.05f   //2024.3.2之前是0.05
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000 //2024.3.2之前是6000
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.003f
#define GIMBAL_INIT_YAW_SPEED   0.006f  //2024.3.2之前是0.006f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

#define YAW_ECD_TO_RAD Motor_Ecd_to_Rad/3

//自瞄滤波值
#define GIMBAL_ACCEL_YAW_NUM 0.12f  //0.02
#define GIMBAL_ACCEL_PITCH_NUM 0.2f

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;     //云台电机结构体
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;    //绝对角度位置pid
    Gimbal_PID_t gimbal_motor_relative_angle_pid;    //相对角度位置pid
    PidTypeDef gimbal_motor_gyro_pid;                //yaw电机 速度电流pid
    gimbal_motor_mode_e last_gimbal_motor_mode;
    gimbal_motor_mode_e gimbal_motor_mode;           //云台控制状态机
    uint16_t offset_ecd;     //yaw电机转子中间位置
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    first_order_filter_type_t gimbal_cmd_slow_set;    //一阶低通滤波上位机期望

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} Gimbal_Motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;    //遥控器结构体指针
    const ROS_Msg_t *gimbal_ros_msg;    //上位机指令指针
    const fp32 *gimbal_INT_angle_point; //陀螺仪数据指针
    const fp32 *gimbal_INT_gyro_point;  //加速度计数据指针
    Gimbal_Motor_t gimbal_yaw_motor;    //云台yaw电机结构体
    Gimbal_Motor_t gimbal_pitch_motor;  //云台pitch电机结构体
    Gimbal_Cali_t gimbal_cali;          //校准结果结构体
    int8_t ecd_count;                   //编码计数
    int8_t last_super_channel;          //上一次遥控器开关所在的位置
} Gimbal_Control_t;

extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
extern void GIMBAL_task(void *pvParameters);
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);

extern const Gimbal_Control_t *get_gimbal_control_point(void);
void gimbal_offset_init(void);//手动校正中值以及极限角度值
#endif

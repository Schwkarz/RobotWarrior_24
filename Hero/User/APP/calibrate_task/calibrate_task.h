/**
  ****************************RM Warrior 2023****************************
  * @file       校准设备，包括云台,陀螺仪,加速度计,磁力计,底盘.云台校准是主要计算零点
  * @brief      和最大最小相对角度.陀螺仪校准主要是计算零漂.加速度计和磁力计校准还没有实现
  *             因为加速度计还没有必要去校准,而磁力计还没有用.底盘校准是使M3508进入快速
  *             设置ID模式.
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-25-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis calibration
  *
  @verbatim
  ==============================================================================
  *             使用遥控器进行开始校准
  *             第一步:遥控器的两个开关都打到下
  *             第二步:两个摇杆打成\../,保存两秒.\.代表左摇杆向右下打.
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *                    或者摇杆打成'\/' 开始云台校准
  *                    或者摇杆打成/''\ 开始底盘校准
  *
  *             数据在flash中，包括校准数据和名字 name[3] 和 校准标志位 cali_flag
  *             例如head_cali有八个字节,但它需要12字节在flash,如果它从0x080A0000开始
  *             0x080A0000-0x080A0007: head_cali数据
  *             0x080A0008: 名字name[0]
  *             0x080A0009: 名字name[1]
  *             0x080A000A: 名字name[2]
  *             0x080A000B: 校准标志位 cali_flag,当校准标志位为0x55,意味着head_cali已经校准了
  *             添加新设备
  *             1.添加设备名在calibrate_task.h的cali_id_e, 像
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. 添加数据结构在 calibrate_task.h, 必须4字节倍数，像
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //长度:8字节 8 bytes, 必须是 4, 8, 12, 16...
  *             3.在 "FLASH_WRITE_BUF_LENGHT",添加"sizeof(xxx_cali_t)", 和实现新函数
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), 添加新名字在 "cali_name[CALI_LIST_LENGHT][3]"
  *             和申明变量 xxx_cali_t xxx_cail, 添加变量地址在cali_sensor_buf[CALI_LIST_LENGHT]
  *             在cali_sensor_size[CALI_LIST_LENGHT]添加数据长度, 最后在cali_hook_fun[CALI_LIST_LENGHT]添加函数
  *
  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "main.h"

#define imu_start_buzzer() buzzer_on(84, 20) //IMU元件校准的蜂鸣器的频率以及强度

#define gimbal_start_buzzer() buzzer_on(54, 60) //云台校准的蜂鸣器的频率以及强度

#define cali_buzzer_off() buzzer_off() //关闭蜂鸣器

#define cali_get_mcu_temperature() get_temprate() //获取stm32上的温度 作为IMU校准的环境温度

#define cali_flash_read(address, buf, len) flash_read((address), (buf), (len))                  //flash 读取函数
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len)) //flash 写入函数

#define get_remote_ctrl_point_cali() get_remote_control_point() //获取遥控器的结构体指针
#define gyro_cali_disable_control() RC_unable()                 //陀螺仪校准时候掉线遥控器
#define gyro_cali_enable_control()  RC_restart(SBUS_RX_BUF_NUM)

//陀螺仪校准函数
#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//设置陀螺仪校准值
#define gyro_set_cali(cali_scale, cali_offset) INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR ADDR_FLASH_SECTOR_9 //要写入flash扇区地址

#define GYRO_CONST_MAX_TEMP 45.0f //陀螺仪控制恒温 最大控制温度

#define CALI_FUNC_CMD_ON 1   //校准函数，使能校准
#define CALI_FUNC_CMD_INIT 0 //校准函数，传递flash中的校准参数

#define CALIBRATE_CONTROL_TIME 1 //校准任务函数运行的周期 为1ms

#define CALI_SENSOR_HEAD_LEGHT 1 //校准结构体的表头 为cali_sensor_t的前部 详看 cali_sensor_t的描述 大小 1 代表一个32位数据

#define SELF_ID 0              //表头中的ID
#define FIRMWARE_VERSION 12345 //表头中的硬件版本号 目前随意写的
#define CALIED_FLAG 0x55       //代表标记已经校准完毕

#define CALIBRATE_END_TIME 20000 //遥控器校准时长20s 超过20s需要重新操作

#define RC_CALI_BUZZER_MIDDLE_TIME 10000 //校准10s时，改变蜂鸣器频率成云台的高频声音，有助于提醒20s校准时间马上完毕
#define rc_cali_buzzer_middle_on() gimbal_start_buzzer()
#define RC_CALI_BUZZER_START_TIME 0 //校准时，改变蜂鸣器频率成IMU的低频声音，代表校准时间20s开始计时
#define rc_cali_buzzer_start_on() imu_start_buzzer()
#define RCCALI_BUZZER_CYCLE_TIME 400  //校准选择时间20s，蜂鸣器断续发声周期时间
#define RC_CALI_BUZZER_PAUSE_TIME 200 //校准选择时间20s，蜂鸣器断续发声停声时间
#define RC_CALI_VALUE_HOLE 600        //遥控器外八或者内八 阈值判定， 遥控器摇杆最大是 660 只要大于630 就认为到最大

#define RC_CMD_LONG_TIME 2000 //遥控器使能校准的时间，即保持内八的时间

#define GYRO_CALIBRATE_TIME 20000 //陀螺仪校准的时间 20s

//校准设备名
typedef enum
{
    CALI_HEAD,
    CALI_GIMBAL,
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    CALI_LIST_LENGHT,
    //add more...
} cali_id_e;

//校准设备前部，通过flash_buf链接到对应的校准设备变量地址
typedef __packed struct
{
    uint8_t name[3];                                 //device name
    uint8_t cali_done;                               //0x55 means has been calibrated
    uint8_t flash_len : 7;                           //buf lenght
    uint8_t cali_cmd : 1;                            //1 means to run cali hook function
    uint32_t *flash_buf;                             //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd); //cali function
} cali_sensor_t;

//头设备的校准数据
typedef __packed struct
{
    uint8_t self_id;                
    int8_t temperate;           
    uint16_t firmware_version;
    fp32 latitude;
} head_cali_t;
//云台设备的校准数据
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;
//陀螺仪，加速度计，磁力计通用校准数据
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

//初始化，以及读取flash校准值
extern void cali_param_init(void);
//返回mpu6500控制的温度
extern int8_t get_control_temperate(void);
//校准任务
extern void calibrate_task(void *pvParameters);
#endif

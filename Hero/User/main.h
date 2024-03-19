#ifndef MAIN_H
#define MAIN_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

//云台电机可能can发送失败的情况，尝试使用 随机延迟发送控制指令的方式解决
#define GIMBAL_MOTOR_6020_CAN_LOSE_SOLVE 0

#define SysCoreClock 180

#define RC_NVIC 4
#define SPI1_RX_NVIC 5
#define CAN1_NVIC 3
#define CAN2_NVIC 3
#define ROS_NVIC 4
#define REFEREE_NVIC 4
#define IST8310_NVIC 5
#define BMI088_NVIC 5
#define SoftWare_NVIC 5
#define Camera_NVIC 3
#define TIM3_NVIC 1

#define temperature 26

#define Latitude_At_ShenZhen 22.57025f

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

#endif /* __MAIN_H */

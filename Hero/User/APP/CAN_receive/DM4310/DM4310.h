#ifndef DM4310_H
#define DM4310_H
#include "main.h"

#define DM_CAN CAN2

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE	    	0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

//达妙电机数据获取
#define get_DMMotor_measure(ptr, rx_message)                                                       \
    {                                                                                              \
        (ptr)->id = (uint8_t)((rx_message)->Data[0] & 0x0f);                                       \
        (ptr)->err = (uint8_t)((rx_message)->Data[0] >> 4);                                      \
        (ptr)->pos_int = (uint16_t)((rx_message)->Data[1] << 8 | (rx_message)->Data[2]);           \
        (ptr)->vel_int = (uint16_t)((rx_message)->Data[3] << 4 | (rx_message)->Data[4] >> 4);      \
        (ptr)->tor_int = (uint16_t)(((rx_message)->Data[4] & 0x0f) << 8 | (rx_message)->Data[5]);  \
        (ptr)->T_Mos = (float)((rx_message)->Data[6]);                                             \
        (ptr)->T_Rotor = (float)((rx_message)->Data[7]);                                           \
    }

typedef struct 
{
    uint8_t id;             // 控制器id
    uint8_t err;            // 故障类型
    uint16_t pos_int;
    uint16_t vel_int;
    uint16_t tor_int;
    float pos;              // 位置信息
    float vel;              // 速度信息
    float tor;              // 扭矩信息
    float T_Mos;            // MOS管上的温度信息
    float T_Rotor;          // 电机线圈的温度信息
} DMMotor_measure_t;

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void enable_motor_mode(uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(uint16_t motor_id, uint16_t mode_id);
void save_pos_zero(uint16_t motor_id, uint16_t mode_id);

void mit_ctrl(uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);

#endif
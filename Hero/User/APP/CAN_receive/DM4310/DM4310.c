#include "DM4310.h"
#include "stm32f4xx.h"


int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

// 使能DM电机
void enable_motor_mode(uint16_t motor_id, uint16_t mode_id)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = motor_id + mode_id;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFC;

    CAN_Transmit(DM_CAN, &TxMessage);
}

// 失能DM电机
void disable_motor_mode(uint16_t motor_id, uint16_t mode_id)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = motor_id + mode_id;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFD;

    CAN_Transmit(DM_CAN, &TxMessage);
}

void save_pos_zero(uint16_t motor_id, uint16_t mode_id)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = motor_id + mode_id;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0xFF;
    TxMessage.Data[1] = 0xFF;
    TxMessage.Data[2] = 0xFF;
    TxMessage.Data[3] = 0xFF;
    TxMessage.Data[4] = 0xFF;
    TxMessage.Data[5] = 0xFF;
    TxMessage.Data[6] = 0xFF;
    TxMessage.Data[7] = 0xFE;

    CAN_Transmit(DM_CAN, &TxMessage);
}

/**
  * @brief:       mit_ctrl: MIT模式下的电机控制函数
  * @param[in]:   hcan:	    指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
  * @param[in]:   motor_id:	电机ID，指定目标电机
  * @param[in]:   pos:		位置给定值
  * @param[in]:   vel:		速度给定值
  * @param[in]:   kp:		位置比例系数
  * @param[in]:   kd:		位置微分系数
  * @param[in]:   torq:		转矩给定值
  * @retval:      void
  * @details:     通过CAN总线向电机发送MIT模式下的控制帧。
  */
void mit_ctrl(uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

    CanTxMsg TxMessage;
    TxMessage.StdId = id;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = (pos_tmp >> 8);
    TxMessage.Data[1] = pos_tmp;
    TxMessage.Data[2] = (vel_tmp >> 4);
    TxMessage.Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    TxMessage.Data[4] = kp_tmp;
    TxMessage.Data[5] = (kd_tmp >> 4);
    TxMessage.Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    TxMessage.Data[7] = tor_tmp;
	
    CAN_Transmit(DM_CAN, &TxMessage);
}

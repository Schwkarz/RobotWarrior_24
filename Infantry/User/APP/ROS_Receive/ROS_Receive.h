#ifndef ROS_RECEIVE_H
#define ROS_RECEIVE_H

#include "main.h"

#define ROS_START_BYTE 0x42
#define ROS_RX_BUF_NUM 32u
#define ROS_FRAME_LENGTH 16u

//陀螺仪数据发送周期 ms
#define IMU_SEND_TIME 20

typedef struct
{
	fp32 shoot_yaw;
	fp32 shoot_pitch;
	fp32 shoot_depth;
}ROS_Msg_t;

void imuSendTask(void *pvParameters);

void ROS_Init(void);
const ROS_Msg_t *get_ROS_Msg_point(void);
void Get_Gimbal_Angle(fp32 *yaw_add,fp32 *pitch_add);

#endif

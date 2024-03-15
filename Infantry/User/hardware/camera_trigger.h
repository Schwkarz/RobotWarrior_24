#ifndef CAMERA_TRIGGER_H
#define CAMERA_TRIGGER_H

#include "main.h"

void camera_trigger_Init(void);
void Camera_trigger_set(uint16_t time_ms);
void Camera_trigger_start(void);
void Camera_trigger_stop(void);



#endif

#ifndef BUZZER_H
#define BUZZER_H
#include "main.h"

void buzzer_init(uint16_t arr, uint16_t psc);
void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);

#endif

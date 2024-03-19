#ifndef ADC_H
#define ADC_H
#include "main.h"

extern void temp_ADC_init(void);
extern void voltage_ADC_init(void);
extern void init_vrefint_reciprocal(void);
extern fp32 get_temprate(void);
extern fp32 get_battery_voltage(void);

#endif

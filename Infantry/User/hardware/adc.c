#include "adc.h"
#include "delay.h"
#include "stm32f4xx.h"

static uint16_t get_ADC1(uint8_t ch);
static uint16_t get_ADC3(uint8_t ch);
static void temperature_ADC_Reset(void);
static void voltage_ADC_Reset(void);

volatile fp32 voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;// 3.3f/4096.0f

void temp_ADC_init(void)
{
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);

    ADC_TempSensorVrefintCmd(ENABLE);//使能内部温度传感器

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟 5 个时钟
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);
    
    temperature_ADC_Reset();

    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 2, ADC_SampleTime_15Cycles);

    ADC_Cmd(ADC1, ENABLE);

}

//ADC_BAT PF10
void voltage_ADC_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3, DISABLE);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);

    voltage_ADC_Reset();

    ADC_RegularChannelConfig(ADC3, ADC_Channel_8, 1, ADC_SampleTime_15Cycles);
    ADC_Cmd(ADC3, ENABLE);

}

static void temperature_ADC_Reset(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 2;
    ADC_Init(ADC1, &ADC_InitStructure);
}

static void voltage_ADC_Reset(void)
{
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC3, &ADC_InitStructure);
}

static uint16_t get_ADC1(uint8_t ch)
{

    ADC_ClearFlag(ADC1,ADC_FLAG_STRT|ADC_FLAG_OVR|ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_15Cycles);

    ADC_SoftwareStartConv(ADC1);

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
    {
        ;
    }
    return ADC_GetConversionValue(ADC1);
}

static uint16_t get_ADC3(uint8_t ch)
{

    ADC_ClearFlag(ADC3,ADC_FLAG_STRT|ADC_FLAG_OVR|ADC_FLAG_EOC);
    ADC_RegularChannelConfig(ADC3, ch, 1, ADC_SampleTime_15Cycles);

    ADC_SoftwareStartConv(ADC3);

    while (!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC))
    {
        ;
    }
    return ADC_GetConversionValue(ADC3);
}

void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += get_ADC1(ADC_Channel_Vrefint);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}

// temperate = (adc - 0.76f) * 400.0f + 25.0f
fp32 get_temprate(void)
{
    uint16_t adcx = 0;
    fp32 temperate = 0;
    temperature_ADC_Reset();
    adcx = get_ADC1(ADC_Channel_TempSensor);
    // temperate = (fp32)adcx * (3.3f / 4096.0f);//不使用VREFINT校准
    temperate = (fp32)adcx * voltage_vrefint_proportion;//使用VREFINT校准
    temperate = (temperate - 0.76f) / 0.0025f + 25.0f;
    return temperate;
}

// 分压的电阻值为 200KΩ和 22KΩ，(22K Ω + 200K Ω) / 22K Ω = 10.09，
fp32 get_battery_voltage(void)
{
    uint16_t adcx = 0;
    fp32 voltage = 0;
    voltage_ADC_Reset();
    adcx = get_ADC3(ADC_Channel_8);
    voltage = (fp32)adcx * voltage_vrefint_proportion;
    voltage = voltage * 10.090909090909090909090909090909f;
    return voltage;
}


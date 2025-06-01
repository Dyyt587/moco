
#include "stspin32g4.h"

// #include "main.h"
#include "cmsis_compiler.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_adc.h"
#include "parameters.h"
#include "motion.h"
#include "dwt.h"
#include <math.h>
#include <tim.h>
#include <adc.h>
#include <opamp.h>
#include <perf_counter.h>

#define GET_PWM_CCR(dc) ((GD_PWM_TIM_ARR + 1) * (float)(dc)) // dc = 0 ~ 1.0

void spg4_init(void)
{
    //dwt_init();

    spg4_set_pwm(0,0,0);
	
	
	
	
//	HAL_ADCEx_Calibration_Start( &hadc1, ADC_SINGLE_ENDED);
//	HAL_ADCEx_Calibration_Start( &hadc2, ADC_SINGLE_ENDED);
//	//delay_ms(20);
//	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);
//	__HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);
//	__HAL_ADC_CLEAR_FLAG( &hadc2, ADC_FLAG_JEOC);
	//delay_ms(100);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);

	HAL_ADCEx_InjectedStart_IT( &hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);


    // LL_TIM_EnableAllOutputs(TIM8);
    // LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1);
    // LL_TIM_CC_EnableChannel(TIM8, LL_TIM_CHANNEL_CH1N);
    // LL_TIM_EnableCounter(TIM8);

//    LL_TIM_EnableIT_UPDATE(ML_TIM_INST);
//    LL_TIM_EnableCounter(ML_TIM_INST);

//    LL_TIM_EnableAllOutputs(GD_PWM_TIM_INST);
//    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH1);
//    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH1N);
//    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH2);
//    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH2N);
//    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH3);
//    LL_TIM_CC_EnableChannel(GD_PWM_TIM_INST, LL_TIM_CHANNEL_CH3N);
//    LL_TIM_EnableCounter(GD_PWM_TIM_INST);

   // spg4_adc_init();

}

uint32_t a = 0;

void ml_handler(void)
{
//    float vm = 12;
//    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_15);
//    open_loop_velocity(0.25F, vm * 0.1F, vm, space_vector_pwm);
}

void spg4_adc_init(void)
{
    // LL_ADC_Enable(VBUS_ADC_INST);
    // LL_ADC_Disable(VBUS_ADC_INST);
    //  while (LL_ADC_IsEnabled(VBUS_ADC_INST))
    {
        __NOP();
    }
    HAL_Delay(10);

    LL_ADC_StartCalibration(VBUS_ADC_INST, LL_ADC_SINGLE_ENDED);
    while (LL_ADC_IsCalibrationOnGoing(VBUS_ADC_INST))
    {
        __NOP();
    }

    // HAL_Delay(LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES);
    HAL_Delay(10);
    LL_ADC_EnableIT_EOC(VBUS_ADC_INST);
    LL_ADC_Enable(VBUS_ADC_INST);
    HAL_Delay(10);

    spg4_adc_start();
}

void spg4_adc_start(void)
{
    LL_ADC_REG_StartConversion(VBUS_ADC_INST);
}

float vbus_adc = 0;
uint16_t adc_1;
uint16_t adc_2;
uint16_t adc_3;
uint16_t adc_4;
uint16_t adc_5;
uint16_t adc_6;

void adc2_isr(void)
{
	
	adc_1 = ADC1->JDR1;
	adc_2 = ADC1->JDR2;
	adc_3 = ADC1->JDR3;
	adc_4 = ADC2->JDR1;
	adc_5 = ADC2->JDR2;
	adc_6 = ADC2->JDR3;
//    if (LL_ADC_IsEnabledIT_EOC(VBUS_ADC_INST))
//    {
//        uint16_t raw = LL_ADC_REG_ReadConversionData32(VBUS_ADC_INST);
//        vbus_adc = (raw * 3.3 / 4096) * 11.0;

//        LL_ADC_ClearFlag_EOC(VBUS_ADC_INST);
//        spg4_adc_start();
//    }
}

void spg4_set_pwm(float dc_a, float dc_b, float dc_c)
{
    LL_TIM_OC_SetCompareCH1(GD_PWM_TIM_INST, GET_PWM_CCR(dc_a));
    LL_TIM_OC_SetCompareCH2(GD_PWM_TIM_INST, GET_PWM_CCR(dc_b));
    LL_TIM_OC_SetCompareCH3(GD_PWM_TIM_INST, GET_PWM_CCR(dc_c));
}

float get_vbus(void)
{
    LL_ADC_REG_StartConversion(VBUS_ADC_INST);
    uint16_t raw = LL_ADC_REG_ReadConversionData32(VBUS_ADC_INST);
    return (raw * 3.3 / 4096) * 11.0; // 12bit
}

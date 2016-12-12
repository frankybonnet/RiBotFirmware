#include <stm32f10x_gpio.h>
#include "power_management.h"


void InitializeADC() {

    /** this source code come from */
    //http://www.badprog.com/electronics-stm32-using-the-adc-peripheral-with-a-potentiometer

    /// declaring GPIO stuff
    GPIO_InitTypeDef GPIO_InitStructure;
    /// declaring ADC struct
    ADC_InitTypeDef ADC_InitStructure;

    /// deinit ADC
    ADC_DeInit(ADC1);

    /// enabling clock in theory it is already done
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // enabling ADC clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    /// C - Init the GPIO with the structure - Testing ADC
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /// init ADC struct
    ADC_StructInit(&ADC_InitStructure);

    /// setting the ADC struct
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    /// init adc
    ADC_Init(ADC1, &ADC_InitStructure);

    /// enable ADC
    ADC_Cmd(ADC1, ENABLE);

    /// start ADC1 calibration and check the end
    ADC_StartCalibration(ADC1);

    /// configure ADC12_IN0 -> channel 0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);

}


float ReadADC(){
    float valueVolt=0.0;
    /// start ADC -> retrieve the ADC value from the potentiometer
    /// and add it into ADC1->DR with:
    /// ADCx->CR2 |= CR2_EXTTRIG_SWSTART_Set;
    /// this without using ADC1->DR o_O
    /// CR2 = Configuration Register2 -> it seems to be a config with
    /// a binary number -> for example 1000010100101010 which sets all
    /// registers with default values
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /// check the end of the ADC1 calibration
    /// setting ADC1->DR with:
    /// if ((ADCx->CR2 & CR2_CAL_Set) != (uint32_t)RESET)
    while (ADC_GetCalibrationStatus(ADC1) == SET)
        ;

    // convert valueADC to valueVolt -> valueADC * (MAX VOLT / 2^12)
    // and also :
    // ADC_SoftwareStartConvCmdsoftwareS,
    // ADC_GetCalibrationStatus
    // with
    // return (uint16_t) ADCx->DR;
    uint16_t valueADC = ADC_GetConversionValue(ADC1);

    // convert the "uint_16 valueADC" into a "float valueVolt"
    // Volt = 3.3
    // ADC = 12 bits, so 2^12 = 4096
    valueVolt = (float)valueADC * 3.3;
    valueVolt=valueVolt/ 4095;
    valueVolt=valueVolt/0.71942; //because of resistive divider

    return valueVolt;

}

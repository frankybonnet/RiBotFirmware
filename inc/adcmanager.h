#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H 120

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_adc.h"
//#include "defines.h"

typedef enum {
	TM_ADC_Channel_0,  /*!< Operate with ADC channel 0 */
	TM_ADC_Channel_1,  /*!< Operate with ADC channel 1 */
	TM_ADC_Channel_2,  /*!< Operate with ADC channel 2 */
	TM_ADC_Channel_3,  /*!< Operate with ADC channel 3 */
	TM_ADC_Channel_4,  /*!< Operate with ADC channel 4 */
	TM_ADC_Channel_5,  /*!< Operate with ADC channel 5 */
	TM_ADC_Channel_6,  /*!< Operate with ADC channel 6 */
	TM_ADC_Channel_7,  /*!< Operate with ADC channel 7 */
	TM_ADC_Channel_8,  /*!< Operate with ADC channel 8 */
	TM_ADC_Channel_9,  /*!< Operate with ADC channel 9 */
	TM_ADC_Channel_10, /*!< Operate with ADC channel 10 */
	TM_ADC_Channel_11, /*!< Operate with ADC channel 11 */
	TM_ADC_Channel_12, /*!< Operate with ADC channel 12 */
	TM_ADC_Channel_13, /*!< Operate with ADC channel 13 */
	TM_ADC_Channel_14, /*!< Operate with ADC channel 14 */
	TM_ADC_Channel_15, /*!< Operate with ADC channel 15 */
	TM_ADC_Channel_16, /*!< Operate with ADC channel 16 */
	TM_ADC_Channel_17, /*!< Operate with ADC channel 17 */
	TM_ADC_Channel_18  /*!< Operate with ADC channel 18 */
} TM_ADC_Channel_t;

/**
 * @}
 */

/**
 * @defgroup TM_ADC_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes ADCx peripheral
 * @param  *ADCx: ADCx peripheral to initialize
 * @retval None
 */
void TM_ADC_InitADC(ADC_TypeDef* ADCx);

/**
 * @brief  Initializes ADCx with ADCx channel
 * @param  *ADCx: ADCx peripheral to operate with
 * @param  channel: channel for ADCx
 * @retval None
 */
void TM_ADC_Init(ADC_TypeDef* ADCx, uint8_t channel);

/**
 * @brief  Reads from ADCx channel
 * @param  *ADCx: ADCx peripheral to operate with
 * @param  channel: channel for ADCx to read from
 * @retval ADC value
 */
uint16_t TM_ADC_Read(ADC_TypeDef* ADCx, uint8_t channel);

/**
 * @brief  Enables Vbat channel for ADC
 * @param  None
 * @retval None
 */
void TM_ADC_EnableVbat(void);

/**
 * @brief  Disables Vbat channel for ADC
 * @param  None
 * @retval None
 */
void TM_ADC_DisableVbat(void);

/**
 * @brief  Reads vbat pin voltage
 * @param  *ADCx: ADCx peripheral to use for Vbat measurement
 * @retval voltage in mV
 */
uint16_t TM_ADC_ReadVbat(ADC_TypeDef* ADCx);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif

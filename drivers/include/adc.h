﻿/*!****************************************************************************
 * @file    	adc.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	07.03.2017
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"

/*!****************************************************************************
* User define
*/
#define ADC_NUM_CH                  (3)
#define ADC_TIM_FREQUENCY        	(24000000)  //[Hz]
#define SDADC_DR_TO_LSB_ADD     	32767

/*!****************************************************************************
* User typedef
*/
typedef struct adcStct{
    uint16_t        sampleRate;                         ///<[us]
    uint16_t        adcreg[ADC_NUM_CH];                 ///<
    void (*tcHoock)(struct adcStct *adc);
}adcStct_type;

typedef void (*adcCallback_type)(adcStct_type *adc);

/*!****************************************************************************
* User enum
*/
enum{
	CH_UINADC,
    CH_IADC,
	CH_UADC,
};

/*!****************************************************************************
* Prototypes for the functions
*/
void adc_init(void);
void adc_startSampling(void);
void adc_stopSampling(void);
void adc_setSampleRate(uint16_t us);
void adc_setCallback(adcCallback_type tcHoock);

#ifdef __cplusplus
}
#endif

#endif //ADC_H
/******************************** END OF FILE ********************************/
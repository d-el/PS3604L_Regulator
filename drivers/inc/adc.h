/*!****************************************************************************
* @file    		adc.h
* @author  		Storozhenko Roman - D_EL
* @version 		V1.0
* @date    		07.03.2017
* @copyright 	GNU Public License
*/
#ifndef ADC_H
#define ADC_H

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"
#include "gpio.h"
#include "board.h"
#include "OSinit.h"

/*!****************************************************************************
* User define
*/
#define ADC_NUM_CH                  (3)
#define ADC_TIM_FREQUENCY        	(24000000)  //[Hz]
#define SDADC_DR_TO_LSB_ADD     	32767

/*!****************************************************************************
* User typedef
*/
typedef struct{
    uint16_t        sampleRate;                         ///<[us]
    uint16_t        adcreg[ADC_NUM_CH];                 ///<
}adcStct_type;

/*!****************************************************************************
* User enum
*/
enum{
    CH_UADC,
	CH_UINADC,
    CH_IADC,
};

/*!****************************************************************************
* External variables
*/
extern adcStct_type     adcStct;

/*!****************************************************************************
* Macro functions
*/
#define startSampling()             TIM3->CR1 |= TIM_CR1_CEN
#define stopSampling()              TIM3->CR1 &= ~TIM_CR1_CEN
#define setSampleRate(x)			TIM3->ARR = (x)

/*!****************************************************************************
* Prototypes for the functions
*/
void sdadc_init(void);
void DMA1_Channel1_IRQHandler(void);
void adcCalc(void);

#endif //ADC_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

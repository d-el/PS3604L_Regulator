/*!****************************************************************************
* @file			board.h
 * @author		Storozhenko Roman - D_EL
* @version 		V1.0
* @date			10.01.2021, Storozhenko Roman
* @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
*
* TIM12_CH1 (PB14)				-> LED
* TIM3_CH1 						-> ADC TRIGGER
* TIM4_CH2 (PA12)				-> FAN_PWM
*
* SDADC1_IN4P 					-> UDC_MEAS
* SDADC1_IN5P 					-> I_MEAS
* SDADC1_IN6P 					-> U_MEAS
*
* UART3 TX - (PD8)				-> 1Wire
* UART1 TX - (PA9), RX - (PA10)	-> UART CONNECT
*
* GPIO (PB5) 					-> ON_OFF
* GPIO (PA0)					-> CC_CV
*/
#ifndef board_H
#define board_H

/*!****************************************************************************
* Include
*/
#include "dac.h"
#include "gpio.h"

/*!****************************************************************************
* User define
*/
#define SYSTEM_FREQ			64000000	//[Hz]
#define APB1_FREQ			32000000	//[Hz]
#define APB1_TIM_FREQ		64000000	//[Hz]
#define APB2_FREQ			32000000	//[Hz]
#define APB2_TIM_FREQ		64000000	//[Hz]

// Task Priority
#define EXTI_CC_CV_Priority			0
#define PVD_IRQ_Priority			1
#define DMA2_Channel3_IRQn_Priority	15

/*!****************************************************************************
* Macro functions
*/
#define setDacI(u16val)		setDacCh1(u16val)
#define setDacU(u16val)		setDacCh2(u16val)
#define LED_ON()			gppin_set(GP_LED)
#define LED_OFF()			gppin_reset(GP_LED)
#define LED_TOGGLE()		gppin_togle(GP_LED)
#define SWITCH_OFF()		_gppin_set(GPIOB, pinm9)
#define MODE_IS_CC()		((gppin_get(GP_CC_CV)) ? 1:0)   //Определяет проверка режима ограничения

/*!****************************************************************************
* Prototypes for the functions
*/

#endif //board_H
/******************************** END OF FILE ********************************/

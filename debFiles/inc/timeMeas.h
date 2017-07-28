/*!****************************************************************************
* @file    		timeMeas
* @author  		d_el
* @version 		V1.0
* @date    		04.10.2015, by d_el
* @brief   		debug
* @copyright 	GNU Public License
*/
#ifndef timeMeas_H
#define timeMeas_H

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"

/*!****************************************************************************
* User define
*/
#define NUMTIMERS       4
#define MEASTIM         TIM2
#define MEASTIM_FREQ    24000000

/*!****************************************************************************
* User typedef
*/

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* Extern viriables
*/
extern unsigned long long    measTime[NUMTIMERS];

/*!****************************************************************************
* Macro functions
*/
//#define measTimeStart(){                                \
//    MEASTIM->CR1 |= TIM_CR1_CEN;                        \
//    MEASTIM->CNT = 0;                                   \
//}
//#define measTimeStop(n){                                \
//    measTime[n] = MEASTIM->CNT;                         \
//    MEASTIM->CR1 &= ~TIM_CR1_CEN;                       \
//    /*measTime[n] = (measTime[n] * 1000000000) / MEASTIM_FREQ; */\
//        measTime[n] = MEASTIM->CNT;         \
//}

#define measTimeStart() _gppin_set(GPIOA, pinm15);
#define measTimeStop(n) _gppin_reset(GPIOA, pinm15);

/*!****************************************************************************
* Prototypes for the functions
*/
void timMeasInit(void);

#endif //timeMeas_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

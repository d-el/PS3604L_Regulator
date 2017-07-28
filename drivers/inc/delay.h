/*!****************************************************************************
* @file    		delay
* @author  		Storozhenko Roman - D_EL
* @version 		V2.2
* @date    		17-01-2015
* @copyright 	GNU Public License
*/
#ifndef delay_H
#define delay_H

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"
#include "stdint.h"

/*!****************************************************************************
* User define
*/
#define DELTIM_FREQ     			(24000000)
#define DELTIM          			(TIM7)
#define DELTIM_USE_STATIC_INLINE	(1)

/*!****************************************************************************
* User typedef
*/

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* Extern viriables
*/

/*!****************************************************************************
* Macro functions
*/
#define delTim_init()                                    \
{                                                       \
    RCC->APB1ENR    |= RCC_APB1ENR_TIM7EN;             	\
}

#if(DELTIM_USE_STATIC_INLINE > 0)

/*!****************************************************************************
* @brief
*/
__attribute__((always_inline)) __STATIC_INLINE
void delay_us(uint16_t us){
    DELTIM->PSC     = DELTIM_FREQ / 1000000 - 1;
    DELTIM->ARR     = us;
    DELTIM->EGR     = TIM_EGR_UG;                                   //Update generation
    __DSB();														//Data Synchronization Barrier
    DELTIM->SR      = ~TIM_SR_UIF;
    DELTIM->CR1     = TIM_CR1_OPM | TIM_CR1_CEN;                   	//Counter enable
    while((DELTIM->SR & TIM_SR_UIF) == 0);
}

/*!****************************************************************************
* @brief
*/
__attribute__((always_inline)) __STATIC_INLINE
void delay_ms(uint16_t ms){
    DELTIM->PSC     = DELTIM_FREQ / 1000 - 1;
    DELTIM->ARR     = ms;
    DELTIM->EGR     = TIM_EGR_UG;                                   //Update generation
    __DSB();														//Data Synchronization Barrier
    DELTIM->SR      = ~TIM_SR_UIF;
    DELTIM->CR1     = TIM_CR1_OPM | TIM_CR1_CEN;                   	//Counter enable
    while((DELTIM->SR & TIM_SR_UIF) == 0);
}

#endif  //#if(DELTIM_USE_STATIC_INLINE > 0)

/*!****************************************************************************
* Prototypes for the functions
*/
#if(DELTIM_USE_STATIC_INLINE == 0)
void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
#endif  //#if(DELTIM_USE_STATIC_INLINE == 0)

#endif //delay_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

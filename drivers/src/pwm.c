/*!****************************************************************************
* @file    		pwm.c
* @author  		d_el
* @version 		V1.0
* @date   	 	02.05.2016, Storozhenko Roman
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"
#include "board.h"
#include "gpio.h"
#include "pwm.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
* @brief    TIM5_CH4 - PB9
*/
void pwmFan_init(void){
    //TIM4_CH2 - PA12
    gppin_init(GPIOA, 12, alternateFunctionPushPull, pullDisable, 0, 10);

    RCC->APB1ENR  	|= RCC_APB1ENR_TIM4EN;                    	//Clock enable
    RCC->APB1RSTR 	|= RCC_APB1RSTR_TIM4RST;              		//Timer reset
    RCC->APB1RSTR 	&= ~RCC_APB1RSTR_TIM4RST;

    TIM4->PSC   	= 1 - 1;                                   	//Prescaler
    TIM4->CCMR1 	|= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;    	//PWM mode 1 (NORMAL PWM)
    TIM4->ARR   	= APB1_FREQ / PWM_FAN_FREQ;                	//Auto reload register
    TIM4->CCR2  	= 0;                                       	//Compare value
    TIM4->CR1   	|= TIM_CR1_ARPE;                           	//TIMx_ARR register is buffered
    TIM4->CCER		|= TIM_CCER_CC2E;							//CH2 Output Enable
    TIM4->CR1   	|= TIM_CR1_CEN;                            	//Counter enable
}

/*!****************************************************************************
* @brief    set duty cycle  [X_XX %]
*/
void FanPwmSet(uint16_t dc){
	if(dc > 1000)
		dc = 1000;
	TIM4->CCR2 = ((uint32_t)TIM4->ARR * dc) / 1000;
}


/*!****************************************************************************
* @brief    TIM15_CH2 - PB15
*/
void Init_PwmLed(void){
    //TIM12_CH1 - PB14
    gppin_init(GPIOB, 14, alternateFunctionPushPull, pullDisable, 0, 9);

    RCC->APB1ENR  |= RCC_APB1ENR_TIM12EN;                    	//Clock enable
    RCC->APB1RSTR |= RCC_APB1RSTR_TIM12RST;                  	//Timer reset
    RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM12RST;

    TIM12->PSC   = 1 - 1;                                    	//Prescaler
    TIM12->CCER   |= TIM_CCER_CC2E;                       		//Channel 2 enable
    TIM12->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;     	//PWM mode 1 (NORMAL PWM)
    TIM12->ARR   = APB1_FREQ / PWM_LED_FREQ;                 	//Auto reload register
    TIM12->CCR1  = 0;                                        	//Compare value
    TIM12->CR1   |= TIM_CR1_ARPE;                            	//TIMx_ARR register is buffered
    TIM12->BDTR  |= TIM_BDTR_MOE;                            	//Main output enable
    TIM12->CR1   |= TIM_CR1_CEN;                              	//Counter enable
}

/*!****************************************************************************
* @brief    set duty cycle  [X_XX %]
*/
void LedPwmSet(uint16_t dc){
	if(dc > 1000)
		dc = 1000;
	TIM12->CCR1 = ((uint32_t)TIM12->ARR * dc) / 1000;
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

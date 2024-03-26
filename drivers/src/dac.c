/*!****************************************************************************
 * @file    	dac.c
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	02.05.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include "gpio.h"
#include "dac.h"

/*!****************************************************************************
* MEMORY
*/

/*!****************************************************************************
* @brief    initialization DAC1 CH1, CH2
*/
void dac_init(void){
	gppin_init(GPIOA, 4, analogMode, pullDisable, 0, 0);
	gppin_init(GPIOA, 5, analogMode, pullDisable, 0, 0);

	RCC->APB1ENR	|= RCC_APB1ENR_DAC1EN;				// Clock enable
	RCC->APB1RSTR	|= RCC_APB1RSTR_DAC1RST;			// Reset
	RCC->APB1RSTR	&= ~RCC_APB1RSTR_DAC1RST;
	DAC1->CR		|= DAC_CR_TSEL1 | DAC_CR_TSEL2;
	DAC1->CR		|= DAC_CR_TEN1 | DAC_CR_TEN2;
	DAC1->CR		|= DAC_CR_BOFF1 | DAC_CR_BOFF2;		// Output buffer disable
	DAC1->CR		|= DAC_CR_EN1 | DAC_CR_EN2;			// Enable channel
	DAC1->DHR12R1	= 0;
	DAC1->DHR12R2	= 0;
}

void dac_ch1(uint16_t val){
	DAC->DHR12R1 = val;
	DAC1->SWTRIGR = DAC_SWTRIGR_SWTRIG1;
}

void dac_ch2(uint16_t val){
	DAC->DHR12R2 = val;
	DAC1->SWTRIGR = DAC_SWTRIGR_SWTRIG2;
}

/******************************** END OF FILE ********************************/

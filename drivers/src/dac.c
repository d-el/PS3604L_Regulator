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

	RCC->APB1ENR	|= RCC_APB1ENR_DAC1EN;						// Clock enable
	DAC1->CR		|= DAC_CR_BOFF1;							// DAC channel1 output buffer disable
	DAC1->CR		|= DAC_CR_BOFF2;							// DAC channel2 output buffer disable
	DAC1->CR		|= DAC_CR_EN1;								// Enable channel 1
	DAC1->CR		|= DAC_CR_EN2;								// Enable channel 2
	DAC1->DHR12R1	= 0;
	DAC1->DHR12R2	= 0;
}

/******************************** END OF FILE ********************************/

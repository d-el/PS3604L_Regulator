/*!****************************************************************************
 * @file    	clock.c
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.1
 * @date    	10.01.2022
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"
#include "clock.h"

/*!****************************************************************************
* MEMORY
*/
clock_type clock;

/*!****************************************************************************
*
*/
void clock_init(void){
	uint32_t timeOut;
	useGen_type		useGen;

	RCC->CFGR = RCC_CR_HSEON;

	// Enable HSE
	RCC->CR |= RCC_CR_HSEON;		//HSE oscillator ON
	timeOut = RCC_WAIN_TIMEOUT;
	while(((RCC->CR & RCC_CR_HSERDY) == 0) && (--timeOut != 0));

	// Enable HSE
	if(timeOut != 0){												//HSE is ready
		RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV;					//PLL on HSE
		RCC->CFGR2 = HSE_PLL_PREDIV - 1;							//Div
		RCC->CFGR |= ((HSE_PLL_MUL - 2) << RCC_CFGR_PLLMUL_Pos);	//Mul
		useGen = clock_useHse;
	}

	//Enable HSI
	else{														//HSE is not ready
		RCC->CR &= ~RCC_CR_HSEON;								//HSE oscillator OFF
		RCC->CR |= RCC_CR_HSION;								//HSI oscillator ON
		while((RCC->CR & RCC_CR_HSION) == 0);
		RCC->CFGR |= RCC_CFGR_PLLSRC_HSI_DIV2;					//PLL in HSI/2
		RCC->CFGR |=((HSI_PLL_MUL - 2) << RCC_CFGR_PLLMUL_Pos);	//Mul
		useGen = clock_useHsi;
	}

	// Enable PLL
	RCC->CR  |= RCC_CR_PLLON;				//Enable PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0);	//Wait PLL

	// Set Prescaler
	RCC->CFGR |= RCC_CFGR_PPRE2_2; // APB2 clock divided by 2
	RCC->CFGR |= RCC_CFGR_PPRE1_2; // APB1 clock divided by 2

	// Set Flash latency
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;

	// Switch to PLL
	RCC->CFGR  &= ~RCC_CFGR_SW;					//Clear SW0, SW1
	RCC->CFGR |= RCC_CFGR_SW_PLL;				//Select system clock - PLL
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	//Wait on switch PLL

	clock.useGen = useGen;
}

/******************************** END OF FILE ********************************/

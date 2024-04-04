/*!****************************************************************************
 * @file		gpio.c
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date		22.11.2016
 * @brief		gpio driver for stm32 F3 microcontroller
 * @copyright	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <stddef.h>
#include "stm32f3xx.h"
#include "bitbanding.h"
#include "board.h"
#include "gpio.h"

/*!****************************************************************************
* MEMORY
*/
pinMode_type   const pinsMode[] = {
	/*0 */	makepin(GPIOF,	6,		outPushPull,				pullDisable,	1,	0),	//LED
	/*1 */	makepin(GPIOF,	7,		outPushPull,				pullDisable,	1,	0),	//AD5663_SYNC
	/*2 */	makepin(GPIOB,	7,		outOpenDrain,				pullDisable,	1,	0),	//ON_OFF
	/*3 */	makepin(GPIOA,	0,		digitalInput,				pullDisable,	0,	0),	//CC_CV
	/*4 */	makepin(GPIOA,	2,		alternateFunctionOpenDrain,	pullDisable,	1,	7),	//DS18B20
	/*5 */	makepin(GPIOA,	15,		outPushPull,				pullDisable,	1,	0),	//SPI3_NSS
};
static const uint32_t pinNum = sizeof(pinsMode) / sizeof(pinMode_type);

static gpioCallback_type gpioCallback;

/*!****************************************************************************
* @brief    interrupt handler CC_CV
*/
void EXTI0_IRQHandler(void){
	SWITCH_OFF();
	EXTI->PR = EXTI_PR_PR0;	//Pending
	if(gpioCallback != NULL){
		gpioCallback(NULL);
	}
}

/*!****************************************************************************
* @brief    init CC_CV interrupt
*/
void externalInterrupt_CcCv_init(void){
	EXTI_INIT(GPIOA, 0, EXTI_MODE_RISE, EXTI_CC_CV_Priority);
}

void irqLimitOn(void){
	EXTI->IMR |= EXTI_IMR_MR0;	//Interrupt request from Line x is not masked
}

void irqLimitOff(void){
	EXTI->IMR &= ~EXTI_IMR_MR0;	//Interrupt request from Line x is masked
}

void irqSetCallback(gpioCallback_type callback){
	gpioCallback = callback;
}

/*!****************************************************************************
* InitAllGpio
*/
void gpio_init(void){
	pinMode_type *pgpios;
	pinMode_type *pgpiosEnd;

	pgpios = (pinMode_type*)pinsMode;
	pgpiosEnd = pgpios + pinNum;

	while(pgpios < pgpiosEnd){
		gppin_init(pgpios->p, pgpios->npin, pgpios->mode, pgpios->pull, pgpios->iniState, pgpios->nAF);
		pgpios++;
	}
}

/*!****************************************************************************
*
*/
void gppin_init(GPIO_TypeDef *port, uint8_t npin, gpioMode_type mode, gpioPull_type pull, uint8_t iniState, uint8_t nAF){
	//Clock enable
		 if(port == GPIOA)   RCC->AHBENR    |= RCC_AHBENR_GPIOAEN;
	else if(port == GPIOB)   RCC->AHBENR    |= RCC_AHBENR_GPIOBEN;
	else if(port == GPIOC)   RCC->AHBENR    |= RCC_AHBENR_GPIOCEN;
	else if(port == GPIOD)   RCC->AHBENR    |= RCC_AHBENR_GPIODEN;
	else if(port == GPIOE)   RCC->AHBENR    |= RCC_AHBENR_GPIOEEN;
	else if(port == GPIOF)   RCC->AHBENR    |= RCC_AHBENR_GPIOFEN;
	/*else if(port == GPIOG)   RCC->AHBENR    |= RCC_AHBENR_GPIOGEN;
	else if(port == GPIOH)   RCC->AHBENR    |= RCC_AHBENR_GPIOHEN;*/

	if(iniState != 0){
		port->BSRR = (1<<npin);
	}
	else{
		port->BRR = (1<<npin);
	}

	/*
	* Clear bit field
	*/
	port->MODER			&= ~(0x03 << (2 * npin));
	port->OTYPER		&= ~(1<<npin);
	port->PUPDR			&= ~(GPIO_RESERVED << (2*npin));
	port->AFR[npin / 8] &= ~(GPIO_AFRL_AFRL0_Msk << (4*(npin % 8)));

	//Set pull
	if(pull == pullUp){
		port->PUPDR |= GPIO_PULL_UP << (2*npin);
	}else if(pull == pullDown){
		port->PUPDR |= GPIO_PULL_DOWN << (2*npin);
	}

	//Set mode
	switch(mode){
		case analogMode:
			port->MODER |= GPIO_ANALOG_MODE << (2*npin);
			break;

		case digitalInput:
			port->MODER &= ~(0x03 << (2 * npin));
			break;

		case outPushPull:
			port->MODER |= GPIO_GP_OUT << (2*npin);
			port->OTYPER |= GPIO_PUSH_PULL << npin;
			break;

		case outOpenDrain:
			port->MODER |= GPIO_GP_OUT << (2*npin);
			port->OTYPER |= GPIO_OPEN_DRAIN << npin;
			break;

		case alternateFunctionPushPull:
			port->MODER |= GPIO_AF_MODE << (2*npin);
			port->OTYPER |= GPIO_PUSH_PULL << npin;
			port->OSPEEDR |= 0 << (2*npin);	//High speed
			break;

		case alternateFunctionOpenDrain:
			port->MODER |= GPIO_AF_MODE << (2*npin);
			port->OTYPER |= GPIO_OPEN_DRAIN << npin;
			port->OSPEEDR |= 0 << (2*npin);	//High speed
			break;
	}

	//Set number alternate function
	port->AFR[npin / 8] |= nAF << (4*(npin % 8));
}

/******************************** END OF FILE ********************************/

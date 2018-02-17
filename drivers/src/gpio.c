/*!****************************************************************************
* @file     	gpio.c
* @author   	Storozhenko Roman - D_EL, 14.08.2016 - 4eef
* @version  	V1.0
* @date     	20.07.2016
* @date     	02.08.2016  fix set nAF
* @brief    	gpio driver for stm32 F0 F3 F4 L4 microcontroller
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "stm32f3xx.h"
#include "bitbanding.h"
#include "board.h"
#include "gpio.h"

/*!****************************************************************************
* MEMORY
*/
pinMode_type   const pinsMode[] = {
	/*0 */  makepin(GPIOB,  14,  	outPushPull,				pullDisable,	0,  0),  //LED
	/*1 */  makepin(GPIOB,  9,  	outOpenDrain,				pullDisable,	1,  0),  //ON_OFF
	/*2 */  makepin(GPIOA,  0,  	digitalInput,   			pullDisable,	0,  0),  //CC_CV
	/*3 */  makepin(GPIOD,  8,  	alternateFunctionOpenDrain,	pullDisable,	1,  0),  //DS18B20
};
static const uint32_t pinNum = sizeof(pinsMode) / sizeof(pinMode_type);

/*!****************************************************************************
* @brief    interrupt handler CC_CV
*/
void EXTI0_IRQHandler(void){
	SWITCH_OFF();
	//rg.tf.state.bit.switchIsON = 0;
	//rg.tf.state.bit.ovfCurrent = 1;
	EXTI->PR    |= EXTI_PR_PR0;     //Pending
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
    port->MODER         &= ~(0x03 << (2 * npin));
    port->OTYPER        &= ~(1<<npin);
    port->PUPDR         &= ~(GPIO_RESERVED << (2*npin));
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
            port->MODER         &= ~(0x03 << (2 * npin));
            break;

        case outPushPull:
            port->MODER |= GPIO_GP_OUT << (2*npin);
            port->OTYPER |= GPIO_PUSH_PULL << npin;
            break;

        case outOpenDrain:
            port->MODER     |= GPIO_GP_OUT << (2*npin);
            port->OTYPER    |= GPIO_OPEN_DRAIN << npin;
            break;

       case alternateFunctionPushPull:
            port->MODER     |= GPIO_AF_MODE << (2*npin);
            port->OTYPER    |= GPIO_PUSH_PULL << npin;
            port->OSPEEDR   |= 3 << (2*npin);   //High speed
            break;

        case alternateFunctionOpenDrain:
            port->MODER     |= GPIO_AF_MODE << (2*npin);
            port->OTYPER    |= GPIO_OPEN_DRAIN << npin;
            break;
    }

    //Set number alternate function
    port->AFR[npin / 8] |= nAF << (4*(npin % 8));
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

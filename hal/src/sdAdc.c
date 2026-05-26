/*!****************************************************************************
 * @file		sdAdc.c
 * @author		Storozhenko Roman - D_EL
 * @version		V1.1
 * @date		08.10.2024
 * @copyright	The MIT License (MIT). Copyright (c) 2024 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <stddef.h>
#include "gpio.h"
#include "board.h"
#include "adc.h"

/*!
 * TIM3 -> SDADC1 -> DMA2_Channel3 -> DMA2_Channel3_IRQHandler
 * TIM3 -> SDADC3 -> DMA2_Channel5 -> DMA2_Channel5_IRQHandler
 */

/*!****************************************************************************
 * MEMORY
 */
adcStct_type adcStct = {
	.sampleRate = 10000,	//Default sample Rate
};

/*!****************************************************************************
 *
 */
static void sdadc1_init(void){
	RCC->APB2ENR	|= RCC_APB2ENR_SDADC1EN;					// SDADC1 clock Enable
	RCC->APB2RSTR	|= RCC_APB2RSTR_SDADC1RST;					// SDADC1 reset
	RCC->APB2RSTR	&= ~RCC_APB2RSTR_SDADC1RST;

	PWR->CR |= PWR_CR_SDADC1EN;
	for(int i = 0; i < 360000; i++) __NOP();

	/**********************************
	 * SDADC1
	 */
	SDADC1->CR1		|= SDADC_CR1_JDMAEN;						// The DMA channel is enabled to read regular data
	SDADC1->CR1		&= ~SDADC_CR1_REFV;							// External reference where the VREF pin must be forced externally
	for(int i = 0; i < 360000; i++) __NOP();

	SDADC1->CR2		|= SDADC_CR2_ADON;							// SDADC is enabled

	for(int i = 0; i < 360000; i++) __NOP();

	SDADC1->CR1		|= SDADC_CR1_INIT;							// Enter initialization mode
	while((SDADC1->ISR & SDADC_ISR_INITRDY) == 0) __NOP();		// Wait for The SDADC is in initialization mode

	{
	SDADC1->CONF0R	|= SDADC_CONF0R_SE0;						// Conversions are executed in single-ended zero-volt reference mode
	SDADC1->CONF0R	&= ~SDADC_CONF0R_GAIN0;						// 1x gain
	}

	SDADC1->CONFCHR1 |= 0 << SDADC_CONFCHR1_CONFCH4_Pos;		// Channel uses the configuration specified in SDADC_CONF0R
	SDADC1->CONFCHR1 |= 0 << SDADC_CONFCHR1_CONFCH5_Pos;		// Channel uses the configuration specified in SDADC_CONF0R
	SDADC1->CONFCHR1 |= 0 << SDADC_CONFCHR1_CONFCH6_Pos;		// Channel uses the configuration specified in SDADC_CONF0R
	SDADC1->CONFCHR1 |= 0 << SDADC_CONFCHR1_CONFCH7_Pos;		// Channel uses the configuration specified in SDADC_CONF0R
	SDADC1->CONFCHR2 |= 0 << SDADC_CONFCHR2_CONFCH8_Pos;		// Channel uses the configuration specified in SDADC_CONF0R

	SDADC1->JCHGR	=	SDADC_JCHGR_JCHG_4 |
						SDADC_JCHGR_JCHG_5 |
						SDADC_JCHGR_JCHG_6 |
						SDADC_JCHGR_JCHG_7 |					// Channel is part of the injected group
						SDADC_JCHGR_JCHG_8;						// Channel is part of the injected group

	SDADC1->CR2		|= SDADC_CR2_JEXTEN_0;						// Each rising edge on the selected trigger makes a request to launch a injected conversion
	SDADC1->CR2		|= SDADC_CR2_JEXTSEL_0 |					// Trigger signal selection for launching injected conversions TIM3_CH1
						SDADC_CR2_JEXTSEL_1;

	SDADC1->CR2		&= ~SDADC_CR2_CALIBCNT;						// One calibration sequence will be performed to calculate OFFSET0[11:0]

	SDADC1->CR1		&= ~SDADC_CR1_INIT;							// Exit initialization mode
	while((SDADC1->ISR & SDADC_ISR_INITRDY) != 0);

	SDADC1->CR2		|= SDADC_CR2_STARTCALIB;
	while((SDADC1->ISR & SDADC_ISR_EOCALF) == 0);				// Wait for Calibration has completed and the offsets have been updated
	SDADC1->CLRISR	= SDADC_ISR_CLREOCALF;

	/**********************************
	 * DMA Init
	 */
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;							// DMA Clock Enable
	DMA2_Channel3->CCR = 0;
	DMA2_Channel3->CCR |= DMA_CCR_PL_0;							// Channel priority level - Medium
	DMA2_Channel3->CCR |= DMA_CCR_MSIZE_0;						// Memory size 16 bit
	DMA2_Channel3->CCR |= DMA_CCR_PSIZE_0;						// Peripheral size 16 bit
	DMA2_Channel3->CCR |= DMA_CCR_MINC;							// Memory increment mode enabled
	DMA2_Channel3->CCR &= ~DMA_CCR_PINC;						// Peripheral increment mode disabled
	DMA2_Channel3->CCR |= DMA_CCR_CIRC;							// Circular mode enabled
	DMA2_Channel3->CCR &= ~DMA_CCR_DIR;							// Read from peripheral
	DMA2_Channel3->CCR |= DMA_CCR_TCIE;							// Transfer complete interrupt enable
	DMA2_Channel3->CNDTR = A1CH_NUMBER;							// Number of data
	DMA2_Channel3->CPAR = (uint32_t)&(SDADC1->JDATAR);			// Peripheral address
	DMA2_Channel3->CMAR = (uint32_t)&adcStct.adcdr1[0];			// Memory address
	NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	NVIC_SetPriority(DMA2_Channel3_IRQn, DMA2_Channel3_IRQn_Priority);
	DMA2_Channel3->CCR |= DMA_CCR_EN;
}

/*!****************************************************************************
 *
 */
static void sdadc3_init(void){
	RCC->APB2ENR	|= RCC_APB2ENR_SDADC3EN;					// SDADC3 clock Enable
	RCC->APB2RSTR	|= RCC_APB2RSTR_SDADC3RST;					// SDADC3 reset
	RCC->APB2RSTR	&= ~RCC_APB2RSTR_SDADC3RST;

	PWR->CR |= PWR_CR_SDADC3EN;
	for(int i = 0; i < 360000; i++) __NOP();

	/**********************************
	 * SDADC3
	 */
	SDADC3->CR1		|= SDADC_CR1_JDMAEN;						// The DMA channel is enabled to read regular data
	SDADC3->CR1		&= ~SDADC_CR1_REFV;							// External reference where the VREF pin must be forced externally
	for(int i = 0; i < 360000; i++) __NOP();

	SDADC3->CR2		|= SDADC_CR2_ADON;							// SDADC is enabled

	for(int i = 0; i < 360000; i++) __NOP();

	SDADC3->CR1		|= SDADC_CR1_INIT;							// Enter initialization mode
	while((SDADC3->ISR & SDADC_ISR_INITRDY) == 0) __NOP();		// Wait for The SDADC is in initialization mode

	{
	SDADC3->CONF0R	|= SDADC_CONF0R_SE0;						// Conversions are executed in single-ended zero-volt reference mode
	SDADC3->CONF0R	&= ~SDADC_CONF0R_GAIN0;						// 1x gain
	}

	SDADC3->CONFCHR1 |= 0 << SDADC_CONFCHR1_CONFCH6_Pos;		// Channel uses the configuration specified in SDADC_CONF0R
	SDADC3->CONFCHR1 |= 0 << SDADC_CONFCHR1_CONFCH7_Pos;		// Channel uses the configuration specified in SDADC_CONF0R
	SDADC3->CONFCHR2 |= 0 << SDADC_CONFCHR2_CONFCH8_Pos;		// Channel uses the configuration specified in SDADC_CONF0R

	SDADC3->JCHGR	=	SDADC_JCHGR_JCHG_6 |
						SDADC_JCHGR_JCHG_7 |
						SDADC_JCHGR_JCHG_8;

	SDADC3->CR2		|= SDADC_CR2_JEXTEN_0;						// Each rising edge on the selected trigger makes a request to launch a injected conversion
	SDADC3->CR2		|= SDADC_CR2_JEXTSEL_0 |					// Trigger signal selection for launching injected conversions TIM3_CH1
						SDADC_CR2_JEXTSEL_1;

	SDADC3->CR2		&= ~SDADC_CR2_CALIBCNT;						// One calibration sequence will be performed to calculate OFFSET0[11:0]

	SDADC3->CR1		&= ~SDADC_CR1_INIT;							// Exit initialization mode
	while((SDADC3->ISR & SDADC_ISR_INITRDY) != 0);

	SDADC3->CR2		|= SDADC_CR2_STARTCALIB;
	while((SDADC3->ISR & SDADC_ISR_EOCALF) == 0);				// Wait for Calibration has completed and the offsets have been updated
	SDADC3->CLRISR	= SDADC_ISR_CLREOCALF;

	/**********************************
	 * DMA Init
	 */
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;							// DMA Clock Enable
	DMA2_Channel5->CCR = 0;
	DMA2_Channel5->CCR |= DMA_CCR_PL_0;							// Channel priority level - Medium
	DMA2_Channel5->CCR |= DMA_CCR_MSIZE_0;						// Memory size 16 bit
	DMA2_Channel5->CCR |= DMA_CCR_PSIZE_0;						// Peripheral size 16 bit
	DMA2_Channel5->CCR |= DMA_CCR_MINC;							// Memory increment mode enabled
	DMA2_Channel5->CCR &= ~DMA_CCR_PINC;						// Peripheral increment mode disabled
	DMA2_Channel5->CCR |= DMA_CCR_CIRC;							// Circular mode enabled
	DMA2_Channel5->CCR &= ~DMA_CCR_DIR;							// Read from peripheral
	DMA2_Channel5->CCR |= DMA_CCR_TCIE;							// Transfer complete interrupt enable
	DMA2_Channel5->CNDTR = A3CH_NUMBER;							// Number of data
	DMA2_Channel5->CPAR = (uint32_t)&(SDADC3->JDATAR);			// Peripheral address
	DMA2_Channel5->CMAR = (uint32_t)&adcStct.adcdr3[0];			// Memory address
	NVIC_EnableIRQ(DMA2_Channel5_IRQn);
	NVIC_SetPriority(DMA2_Channel5_IRQn, DMA2_Channel5_IRQn_Priority);
	DMA2_Channel5->CCR |= DMA_CCR_EN;
}

/*!****************************************************************************
 *
 */
void adc_init(void){
	/**********************************
	 * IO
	 */
	gppin_init(GPIOB, 0, analogMode, pullDisable, 0, 0);
	gppin_init(GPIOB, 1, analogMode, pullDisable, 0, 0);
	gppin_init(GPIOB, 2, analogMode, pullDisable, 0, 0);
	gppin_init(GPIOE, 8, analogMode, pullDisable, 0, 0);
	gppin_init(GPIOE, 9, analogMode, pullDisable, 0, 0);

	gppin_init(GPIOB, 14, analogMode, pullDisable, 0, 0);
	gppin_init(GPIOB, 15, analogMode, pullDisable, 0, 0);
	gppin_init(GPIOD, 8, analogMode, pullDisable, 0, 0);

	for(int i = 0; i < 360000; i++) __NOP();

	/**********************************
	 * Clock
	 */
	RCC->APB1ENR	|=	RCC_APB1ENR_PWREN;						// Power interface clock enabled
	RCC->APB2ENR	|=	RCC_APB2ENR_SYSCFGEN;
	for(int i = 0; i < 360000; i++) __NOP();

	RCC->CFGR		&= ~RCC_CFGR_SDADCPRE;
	RCC->CFGR		|=	RCC_CFGR_SDADCPRE_DIV12;				// SDADC CLK divided
	for(int i = 0; i < 360000; i++) __NOP();

	sdadc1_init();
	sdadc3_init();

	/**********************************
	 * TIM Init
	 */
	RCC->APB1ENR	|= RCC_APB1ENR_TIM3EN;						// Enable clock
	RCC->APB1RSTR	|= RCC_APB1RSTR_TIM3RST;					// Timer 3 reset
	RCC->APB1RSTR	&= ~RCC_APB1RSTR_TIM3RST;
	TIM3->PSC		= APB1_TIM_FREQ / 1000000 - 1;				// Set prescaler
	TIM3->CR1		|= TIM_CR1_ARPE;							// TIMx_ARR register is buffered
	TIM3->CCMR1		|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;		// PWM mode 1 (NORMAL PWM)
	TIM3->CCMR2		|= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;		// PWM mode 1 (NORMAL PWM)
	TIM3->CCR1 = 1;
	TIM3->CCR3 = 1;
	TIM3->CCER		|= TIM_CCER_CC1E | TIM_CCER_CC3E;			// Channel enable
	TIM3->ARR		= adcStct.sampleRate;						// SampleRate, [us]
}

/*!****************************************************************************
 *
 */
void adc_startSampling(void){
	TIM3->CR1 |= TIM_CR1_CEN;
}

/*!****************************************************************************
 *
 */
void adc_stopSampling(void){
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

/*!****************************************************************************
 *
 */
void adc_setSampleRate(uint16_t us){
	adcStct.sampleRate = us;
	TIM3->ARR = us;
}

/*!****************************************************************************
 *
 */
void adc_setCallback(adcCallback_type tcHoock){
	adcStct.tcHoock = tcHoock;
}

/*!****************************************************************************
 *---> DMA for SDADC1 Interrupt Handler
 */
void DMA2_Channel3_IRQHandler(void){
	adcStct.adcreg1[0] = adcStct.adcdr1[0] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg1[1] = adcStct.adcdr1[1] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg1[2] = adcStct.adcdr1[2] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg1[3] = adcStct.adcdr1[3] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg1[4] = adcStct.adcdr1[4] + SDADC_DR_TO_LSB_ADD;
	if(adcStct.tcHoock != NULL){
		adcStct.tcHoock(&adcStct);
	}
	DMA2->IFCR = DMA_IFCR_CTCIF3;
}

/*!****************************************************************************
 *---> DMA for SDADC3 Interrupt Handler
 */
void DMA2_Channel5_IRQHandler(void){
	adcStct.adcreg3[0] = adcStct.adcdr3[0] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg3[1] = adcStct.adcdr3[1] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg3[2] = adcStct.adcdr3[2] + SDADC_DR_TO_LSB_ADD;
	DMA2->IFCR = DMA_IFCR_CTCIF5;
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

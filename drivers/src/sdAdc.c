/*!****************************************************************************
* @file			sdAdc.c
* @author		Storozhenko Roman - D_EL
* @version		V1.0
* @date			07.03.2017
* @copyright	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include <stddef.h>
#include "gpio.h"
#include "board.h"
#include "adc.h"

/*!****************************************************************************
* MEMORY
*/
adcStct_type adcStct = {
	.sampleRate = 10000,	//Default sample Rate
};

SDADC_TypeDef* sdadc1;

/*!****************************************************************************
* TIM3 -> SDADC1 -> DMA2_Channel3 -> DMA2_Channel3_IRQHandler
* SDADC1_IN4P
* SDADC1_IN5P
* SDADC1_IN6P
*/
void adc_init(void){
	/**********************************
	 * IO
	 */
	//Analog Input
	gppin_init(GPIOB, 0, analogMode, pullDisable, 0, 0);
	//Analog Input
	gppin_init(GPIOB, 1, analogMode, pullDisable, 0,  0);
	//Analog Input
	gppin_init(GPIOB, 2, analogMode, pullDisable, 0, 0);

	for(int i = 0; i < 360000; i++) __NOP();

	/**********************************
	 * Clock
	 */
	RCC->APB1ENR	|=	RCC_APB1ENR_PWREN;						//Power interface clock enabled
	RCC->APB2ENR	|=	RCC_APB2ENR_SYSCFGEN;
	for(int i = 0; i < 360000; i++) __NOP();

	RCC->CFGR		&= ~RCC_CFGR_SDADCPRE;
	RCC->CFGR		|=	RCC_CFGR_SDADCPRE_DIV48;				//SDADC CLK divided
	for(int i = 0; i < 360000; i++) __NOP();

	RCC->APB2ENR	|= RCC_APB2ENR_SDADC1EN;					//SDADC1 clock Enable
	RCC->APB2RSTR	|= RCC_APB2RSTR_SDADC1RST;					//SDADC1 reset
	RCC->APB2RSTR	&= ~RCC_APB2RSTR_SDADC1RST;

	PWR->CR |= PWR_CR_SDADC1EN;
	for(int i = 0; i < 360000; i++) __NOP();

	/**********************************
	 * SDADC
	 */
	sdadc1 = SDADC1;
	SDADC1->CR1		|= SDADC_CR1_JDMAEN;						//The DMA channel is enabled to read regular data
	SDADC1->CR1		&= ~SDADC_CR1_REFV;							//External reference where the VREF pin must be forced externally
	for(int i = 0; i < 360000; i++) __NOP();

	SDADC1->CR2		|= SDADC_CR2_ADON;							//SDADC is enabled

	for(int i = 0; i < 360000; i++) __NOP();

	SDADC1->CR1		|= SDADC_CR1_INIT;							//Enter initialization mode
	while((SDADC1->ISR & SDADC_ISR_INITRDY) == 0) __NOP();		//Wait for The SDADC is in initialization mode

	{
	SDADC1->CONF0R	|= SDADC_CONF0R_SE0;						//Conversions are executed in single-ended zero-volt reference mode
	SDADC1->CONF0R	|= SDADC_CONF0R_GAIN0;						//0.5x gain
	}

	{
	SDADC1->CONF1R	|= SDADC_CONF0R_SE0;						//Conversions are executed in single-ended zero-volt reference mode
	SDADC1->CONF1R	&= ~SDADC_CONF0R_GAIN0;						//1x gain
	}

	SDADC1->CONFCHR1 |= 1 << SDADC_CONFCHR1_CONFCH4_Pos;		//Channel 4 uses the configuration specified in SDADC_CONF1R
	SDADC1->CONFCHR1 |= 0 << SDADC_CONFCHR1_CONFCH5_Pos;		//Channel 5 uses the configuration specified in SDADC_CONF0R
	SDADC1->CONFCHR1 |= 1 << SDADC_CONFCHR1_CONFCH6_Pos;		//Channel 6 uses the configuration specified in SDADC_CONF1R

	SDADC1->JCHGR	=	SDADC_JCHGR_JCHG_4 |					//Channel 4 is not part of the injected group
						SDADC_JCHGR_JCHG_5 |					//Channel 5 is not part of the injected group
						SDADC_JCHGR_JCHG_6;						//Channel 6 is not part of the injected group

	SDADC1->CR2		|= SDADC_CR2_JEXTEN_0;						//Each rising edge on the selected trigger makes a request to launch a injected conversion
	SDADC1->CR2		|= SDADC_CR2_JEXTSEL_0 |					//Trigger signal selection for launching injected conversions TIM3_CH1
						SDADC_CR2_JEXTSEL_1;

	SDADC1->CR2		|= SDADC_CR2_CALIBCNT_0;					//Two calibration sequences will be performed to calculate OFFSET0[11:0] and OFFSET1[11:0]

	SDADC1->CR1		&= ~SDADC_CR1_INIT;							//Exit initialization mode
	while((SDADC1->ISR & SDADC_ISR_INITRDY) != 0);

	SDADC1->CR2		|= SDADC_CR2_STARTCALIB;
	while((SDADC1->ISR & SDADC_ISR_EOCALF) == 0);				//Wait for Calibration has completed and the offsets have been updated
	SDADC1->CLRISR	= SDADC_ISR_CLREOCALF;

	/**********************************
	 * DMA Init
	 */
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;							//DMA Clock Enable
	DMA2_Channel3->CCR = 0;
	DMA2_Channel3->CCR |= DMA_CCR_PL_0;							//Channel priority level - Medium
	DMA2_Channel3->CCR |= DMA_CCR_MSIZE_0;						//Memory size 16 bit
	DMA2_Channel3->CCR |= DMA_CCR_PSIZE_0;						//Peripheral size 16 bit
	DMA2_Channel3->CCR |= DMA_CCR_MINC;							//Memory increment mode enabled
	DMA2_Channel3->CCR &= ~DMA_CCR_PINC;						//Peripheral increment mode disabled
	DMA2_Channel3->CCR |= DMA_CCR_CIRC;							//Circular mode enabled
	DMA2_Channel3->CCR &= ~DMA_CCR_DIR;							//Read from peripheral
	DMA2_Channel3->CCR |= DMA_CCR_TCIE;							//Transfer complete interrupt enable
	DMA2_Channel3->CNDTR = CH_NUMBER;							//Number of data
	DMA2_Channel3->CPAR = (uint32_t)&(SDADC1->JDATAR);			//Peripheral address
	DMA2_Channel3->CMAR = (uint32_t)&adcStct.adcdr[0];			//Memory address
	NVIC_EnableIRQ(DMA2_Channel3_IRQn);
	NVIC_SetPriority(DMA2_Channel3_IRQn, DMA2_Channel3_IRQn_Priority);
	DMA2_Channel3->CCR |= DMA_CCR_EN;

	/**********************************
	 * TIM Init
	 */
	RCC->APB1ENR	|= RCC_APB1ENR_TIM3EN;						//Enable clock
	RCC->APB1RSTR	|= RCC_APB1RSTR_TIM3RST;					//Timer 3 reset
	RCC->APB1RSTR	&= ~RCC_APB1RSTR_TIM3RST;
	TIM3->PSC		= ADC_TIM_FREQUENCY / 1000000 - 1;			//Set prescaler
	TIM3->CR1		|= TIM_CR1_ARPE;							//TIMx_ARR register is buffered
	TIM3->CR2		|= TIM_CR2_MMS_2;							//Compare - OC1REF signal is used as trigger output (TRGO)
	TIM3->CCMR1		|= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;		//PWM mode 1 (NORMAL PWM)
	TIM3->CCR1 = 1;
	TIM3->CCER		|= TIM_CCER_CC1E;							//Channel enable
	TIM3->ARR		= adcStct.sampleRate;						//SampleRate, [us]
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
*---> DMA for SAADC Interrupt Handler
*/
void DMA2_Channel3_IRQHandler(void){
	adcStct.adcreg[0] = adcStct.adcdr[0] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg[1] = adcStct.adcdr[1] + SDADC_DR_TO_LSB_ADD;
	adcStct.adcreg[2] = adcStct.adcdr[2] + SDADC_DR_TO_LSB_ADD;
	if(adcStct.tcHoock != NULL){
		adcStct.tcHoock(&adcStct);
	}
	DMA2->IFCR = DMA_IFCR_CTCIF3;
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

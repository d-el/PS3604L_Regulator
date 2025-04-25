/*!****************************************************************************
 * @file		startup_stm32f433xx.c
 * @author		d_el - Storozhenko Roman
 * @version		V1.0
 * @date		14.07.2017
 * @copyright	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 * @brief
 */

/*!****************************************************************************
* Include
*/
#include "stdint.h"

extern void __libc_init_array(void);	///Initialization routines
extern void __libc_fini_array(void);	///Cleanup routines
extern void SystemInit(void);			///System initialization function
extern int main(void);					///Entry point for the application

extern uint32_t _estack;				///Highest address of the user mode stack
extern uint32_t _sdata;					///RAM data start
extern uint32_t _edata;					///RAM data end
extern uint32_t _sidata;				///ROM data start
extern uint32_t _sbss;					///RAM bss start
extern uint32_t _ebss;					///RAM bss end

typedef void(*intVector_type)(void);	///Interrupt service routine type
void Reset_Handler(void);

/*!****************************************************************************
 * @brief  Initialization .bss section
 */
void __initializeBss(uint32_t *bssStart, uint32_t *bssEnd){
	uint32_t* pData = bssStart;

	///Verify align by 4
	if(((uint32_t) bssStart % 4) || ((uint32_t) bssEnd % 4)){
		while(1)
			;
	}

	while(pData < bssEnd){
		*pData++ = 0;
	}
}

/*!****************************************************************************
 * @brief  Initialization .data section
 */
void __initializeData(uint32_t *dataStart, uint32_t *dataEnd, uint32_t *src){
	uint32_t *pData = dataStart;

	///Verify align by 4
	if(((uint32_t) dataStart % 4) || ((uint32_t) dataEnd % 4)||((uint32_t) src % 4)){
		while(1)
			;
	}

	while(pData < dataEnd){
		*pData++ = *src++;
	}
}

/*!****************************************************************************
 * @brief	Default interrupt handler
 */
void Default_Handler(void){
	/*
	 * IPSR bit assignments
	 * [8:0]	ISR_NUMBER
	 * This is the number of the current exception:
	 * 0 = Thread mode
	 * 1 = Reserved
	 * 2 = NMI
	 * ...
	 */
	while(1);
}

///Cortex-M4 Processor Exceptions Numbers
void NMI_Handler				(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 2 Cortex-M4 Non Maskable Interrupt									*/
void HardFault_Handler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 3 Cortex-M4 Hard Fault Interrupt									*/
void MemManage_Handler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 4 Cortex-M4 Memory Management Interrupt							*/
void BusFault_Handler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 5 Cortex-M4 Bus Fault Interrupt									*/
void UsageFault_Handler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 6 Cortex-M4 Usage Fault Interrupt									*/
void SVC_Handler				(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 11 Cortex-M4 SV Call Interrupt										*/
void DebugMon_Handler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 12 Cortex-M4 Debug Monitor Interrupt								*/
void PendSV_Handler				(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 14 Cortex-M4 Pend SV Interrupt										*/
void SysTick_Handler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< 15 Cortex-M4 System Tick Interrupt									*/
///STM32 specific Interrupt Numbers
void WWDG_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< Window WatchDog Interrupt											*/
void PVD_IRQHandler				(void) __attribute__((weak, alias ("Default_Handler")));	/*!< PVD through EXTI Line detection Interrupt							*/
void TAMP_STAMP_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< Tamper and TimeStamp interrupts through the EXTI line 19			*/
void RTC_WKUP_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< RTC Wakeup interrupt through the EXTI line 20						*/
void FLASH_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< FLASH global Interrupt												*/
void RCC_IRQHandler				(void) __attribute__((weak, alias ("Default_Handler")));	/*!< RCC global Interrupt												*/
void EXTI0_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< EXTI Line0 Interrupt												*/
void EXTI1_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< EXTI Line1 Interrupt												*/
void EXTI2_TSC_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< EXTI Line2 Interrupt and Touch Sense Controller Interrupt			*/
void EXTI3_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< EXTI Line3 Interrupt												*/
void EXTI4_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< EXTI Line4 Interrupt												*/
void DMA1_Channel1_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA1 Channel 1 Interrupt											*/
void DMA1_Channel2_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA1 Channel 2 Interrupt											*/
void DMA1_Channel3_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA1 Channel 3 Interrupt											*/
void DMA1_Channel4_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA1 Channel 4 Interrupt											*/
void DMA1_Channel5_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA1 Channel 5 Interrupt											*/
void DMA1_Channel6_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA1 Channel 6 Interrupt											*/
void DMA1_Channel7_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA1 Channel 7 Interrupt											*/
void ADC1_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< ADC1 Interrupts													*/
void CAN_TX_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< CAN TX Interrupt													*/
void CAN_RX0_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< CAN RX0 Interrupt													*/
void CAN_RX1_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< CAN RX1 Interrupt													*/
void CAN_SCE_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< CAN SCE Interrupt													*/
void EXTI9_5_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< External Line[9:5] Interrupts										*/
void TIM15_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM15 global Interrupt												*/
void TIM16_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM16 global Interrupt												*/
void TIM17_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM17 global Interrupt												*/
void TIM18_DAC2_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM18 global Interrupt and DAC2 underrun Interrupt					*/
void TIM2_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM2 global Interrupt												*/
void TIM3_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM3 global Interrupt												*/
void TIM4_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM4 global Interrupt												*/
void I2C1_EV_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< I2C1 Event Interrupt & EXTI Line23 Interrupt (I2C1 wakeup)			*/
void I2C1_ER_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< I2C1 Error Interrupt												*/
void I2C2_EV_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< I2C2 Event Interrupt & EXTI Line24 Interrupt (I2C2 wakeup)			*/
void I2C2_ER_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< I2C2 Error Interrupt												*/
void SPI1_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< SPI1 global Interrupt												*/
void SPI2_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< SPI2 global Interrupt												*/
void USART1_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< USART1 global Interrupt & EXTI Line25 Interrupt (USART1 wakeup)	*/
void USART2_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< USART2 global Interrupt & EXTI Line26 Interrupt (USART2 wakeup)	*/
void USART3_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< USART3 global Interrupt & EXTI Line28 Interrupt (USART3 wakeup)	*/
void EXTI15_10_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< External Line[15:10] Interrupts									*/
void RTC_Alarm_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< RTC Alarm (A and B) through EXTI Line 17 Interrupt					*/
void CEC_IRQHandler				(void) __attribute__((weak, alias ("Default_Handler")));	/*!< CEC Interrupt & EXTI Line27 Interrupt (CEC wakeup)					*/
void TIM12_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM12 global interrupt												*/
void TIM13_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM13 global interrupt												*/
void TIM14_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM14 global interrupt												*/
void TIM5_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM5 global Interrupt												*/
void SPI3_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< SPI3 global Interrupt												*/
void TIM6_DAC1_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM6 global and DAC1 underrun error Interrupts						*/
void TIM7_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM7 global Interrupt												*/
void DMA2_Channel1_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA2 Channel 1 global Interrupt									*/
void DMA2_Channel2_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA2 Channel 2 global Interrupt									*/
void DMA2_Channel3_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA2 Channel 3 global Interrupt									*/
void DMA2_Channel4_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA2 Channel 4 global Interrupt									*/
void DMA2_Channel5_IRQHandler	(void) __attribute__((weak, alias ("Default_Handler")));	/*!< DMA2 Channel 5 global Interrupt									*/
void SDADC1_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< ADC Sigma Delta 1 global Interrupt									*/
void SDADC2_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< ADC Sigma Delta 2 global Interrupt									*/
void SDADC3_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< ADC Sigma Delta 1 global Interrupt									*/
void COMP1_2_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< COMP1 and COMP2 global Interrupt									*/
void USB_HP_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< USB High Priority global Interrupt									*/
void USB_LP_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< USB Low Priority global Interrupt									*/
void USBWakeUp_IRQHandler		(void) __attribute__((weak, alias ("Default_Handler")));	/*!< USB Wakeup Interrupt												*/
void TIM19_IRQHandler			(void) __attribute__((weak, alias ("Default_Handler")));	/*!< TIM19 global Interrupt												*/
void FPU_IRQHandler				(void) __attribute__((weak, alias ("Default_Handler")));	/*!< Floating point Interrupt											*/

/*!****************************************************************************
* Interrupt vector table
*/
intVector_type intVector[] __attribute__ ((section (".isr_vector"))) = {
	///Stack
	(intVector_type)&_estack,
	///Cortex-M4 Processor Exceptions
	Reset_Handler,
	NMI_Handler,
	HardFault_Handler,
	MemManage_Handler,
	BusFault_Handler,
	UsageFault_Handler,
	0,
	0,
	0,
	0,
	SVC_Handler,
	DebugMon_Handler,
	0,
	PendSV_Handler,
	SysTick_Handler,
	WWDG_IRQHandler,
	PVD_IRQHandler,
	TAMP_STAMP_IRQHandler,
	RTC_WKUP_IRQHandler,
	FLASH_IRQHandler,
	RCC_IRQHandler,
	EXTI0_IRQHandler,
	EXTI1_IRQHandler,
	EXTI2_TSC_IRQHandler,
	EXTI3_IRQHandler,
	EXTI4_IRQHandler,
	DMA1_Channel1_IRQHandler,
	DMA1_Channel2_IRQHandler,
	DMA1_Channel3_IRQHandler,
	DMA1_Channel4_IRQHandler,
	DMA1_Channel5_IRQHandler,
	DMA1_Channel6_IRQHandler,
	DMA1_Channel7_IRQHandler,
	ADC1_IRQHandler,
	CAN_TX_IRQHandler,
	CAN_RX0_IRQHandler,
	CAN_RX1_IRQHandler,
	CAN_SCE_IRQHandler,
	EXTI9_5_IRQHandler,
	TIM15_IRQHandler,
	TIM16_IRQHandler,
	TIM17_IRQHandler,
	TIM18_DAC2_IRQHandler,
	TIM2_IRQHandler,
	TIM3_IRQHandler,
	TIM4_IRQHandler,
	I2C1_EV_IRQHandler,
	I2C1_ER_IRQHandler,
	I2C2_EV_IRQHandler,
	I2C2_ER_IRQHandler,
	SPI1_IRQHandler,
	SPI2_IRQHandler,
	USART1_IRQHandler,
	USART2_IRQHandler,
	USART3_IRQHandler,
	EXTI15_10_IRQHandler,
	RTC_Alarm_IRQHandler,
	CEC_IRQHandler,
	TIM12_IRQHandler,
	TIM13_IRQHandler,
	TIM14_IRQHandler,
	0,
	0,
	0,
	0,
	TIM5_IRQHandler,
	SPI3_IRQHandler,
	0,
	0,
	TIM6_DAC1_IRQHandler,
	TIM7_IRQHandler,
	DMA2_Channel1_IRQHandler,
	DMA2_Channel2_IRQHandler,
	DMA2_Channel3_IRQHandler,
	DMA2_Channel4_IRQHandler,
	DMA2_Channel5_IRQHandler,
	SDADC1_IRQHandler,
	SDADC2_IRQHandler,
	SDADC3_IRQHandler,
	COMP1_2_IRQHandler,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	USB_HP_IRQHandler,
	USB_LP_IRQHandler,
	USBWakeUp_IRQHandler,
	0,
	TIM19_IRQHandler,
	0,
	0,
	FPU_IRQHandler,
};

/*!****************************************************************************
 * @brief	Program entry point
 */
void Reset_Handler(void){
	volatile uint32_t* VTOR = (uint32_t*)0xE000ED08;
	*VTOR = (uint32_t)&intVector[0];
	__initializeData(&_sdata, &_edata, &_sidata);
	__initializeBss(&_sbss, &_ebss);
	__libc_init_array();
	SystemInit();
	main();
	__libc_fini_array();
	while(1);
}

/******************************** END OF FILE ********************************/

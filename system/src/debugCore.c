﻿/*!****************************************************************************
* @file    debugCore.c
* @author  d_el
* @version V1.0
* @date    13.12.2017, by d_el
* @brief   --
*/

/*!****************************************************************************
* User include
*/
#include "debugCore.h"

/*!****************************************************************************
* Memory
*/

/*!****************************************************************************
* @brief
* @retval 1 - print mode enable
*         0 - print mode disable
*/
uint32_t coreIsInDebugMode(void){
    if((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0){
        return 1;   //print mode enable
    }
    else{
        return 0;   //print mode disable
    }
}

/*!****************************************************************************
* hard fault handler in C,
* with stack frame location as input parameter
* called from HardFault_Handler
*/
void hard_fault_handler_c(unsigned int * stackedContextPtr){
	volatile uint32_t stacked_r0;
    volatile uint32_t stacked_r1;
    volatile uint32_t stacked_r2;
    volatile uint32_t stacked_r3;
    volatile uint32_t stacked_r12;
    volatile uint32_t stacked_lr;
    volatile uint32_t stacked_pc;
    volatile uint32_t stacked_psr;

    stacked_r0  = stackedContextPtr[0];
    stacked_r1  = stackedContextPtr[1];
    stacked_r2  = stackedContextPtr[2];
    stacked_r3  = stackedContextPtr[3];
    stacked_r12 = stackedContextPtr[4];
    stacked_lr  = stackedContextPtr[5];
    stacked_pc  = stackedContextPtr[6];
    stacked_psr = stackedContextPtr[7];

    printp("\n\n[GAME OVER]\n");
    printp("R0 = 0x%008X\n", stacked_r0);
    printp("R1 = 0x%008X\n", stacked_r1);
    printp("R2 = 0x%008X\n", stacked_r2);
    printp("R3 = 0x%008X\n", stacked_r3);
    printp("R12 = 0x%08X\n", stacked_r12);
    printp("LR [R14] = 0x%08X  subroutine call return address\n", stacked_lr);
    printp("PC [R15] = 0x%08X  program counter\n", stacked_pc);
    printp("PSR = 0x%08X\n", stacked_psr);

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    printp("CFSR = 0x%08X\n", SCB->CFSR);

    // Hard Fault Status Register
    printp("HFSR = 0x%08X\n", SCB->HFSR);

    // print Fault Status Register
    printp("DFSR = 0x%08X\n", SCB->DFSR);

    // Auxiliary Fault Status Register
    printp("AFSR = 0x%08X\n", SCB->AFSR);

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    printp("MMFAR = 0x%08X\n", SCB->MMFAR);

    // Bus Fault Address Register
    printp("BFAR = 0x%08X\n", SCB->BFAR);

    asm("BKPT #1");
    while (1);
}

/*!****************************************************************************
*
*/
void HardFault_Handler(void){
	__asm volatile	(
" 		MOVS   R0, #4							\n" /* Determine if processor uses PSP or MSP by checking bit.4 at LR register.		*/
"		MOV    R1, LR							\n"
"		TST    R0, R1							\n"
"		BEQ    _IS_MSP							\n" /* Jump to '_MSP' if processor uses MSP stack.									*/
"_IS_PSP:                                       \n"
"		MRS    R0, PSP							\n" /* Prepare PSP content as parameter to the calling function below.				*/
"		BL	   hard_fault_handler_c      		\n" /* Call 'hardfaultGetContext' passing PSP content as stackedContextPtr value.	*/
"_IS_MSP:										\n"
"		MRS    R0, MSP							\n" /* Prepare MSP content as parameter to the calling function below.				*/
"		BL	   hard_fault_handler_c		        \n" /* Call 'hardfaultGetContext' passing MSP content as stackedContextPtr value.	*/
	::	);
}

/***************** (C) COPYRIGHT ************** END OF FILE ******** d_el ****/

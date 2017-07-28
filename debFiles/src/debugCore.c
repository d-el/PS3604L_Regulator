/*!****************************************************************************
* @file    		debugCore.c
* @author  		d_el
* @version 		V1.0
* @date    		15.01.2016, by d_el
* @copyright 	GNU Public License
*/

#include "stdint.h"
#include "stm32f3xx.h"
#include "semihosting.h"

/*!****************************************************************************
* Memory
*/

/*!****************************************************************************
* @brief
* @retval 1 - debug mode enable
*         0 - debug mode disable
*/
uint32_t coreIsInDebugMode(void){
    volatile uint32_t _DHCSR;
    
    _DHCSR = *(uint32_t*)0xE000EDF0;    //Debug Halting Control and Status Register
    
    if((_DHCSR & (1<<0)) != 0){
        return 1;   //Debug mode enable
    }
    else{
        return 0;   //Debug mode disable
    }
}

/*!****************************************************************************
* hard fault handler in C,
* with stack frame location as input parameter
* called from HardFault_Handler
*/
void hard_fault_handler_c(unsigned int * stackedContextPtr){
	volatile unsigned long stacked_r0;
    volatile unsigned long stacked_r1;
    volatile unsigned long stacked_r2;
    volatile unsigned long stacked_r3;
    volatile unsigned long stacked_r12;
    volatile unsigned long stacked_lr;
    volatile unsigned long stacked_pc;
    volatile unsigned long stacked_psr;
    volatile unsigned long _CFSR;
    volatile unsigned long _HFSR;
    volatile unsigned long _DFSR;
    volatile unsigned long _AFSR;
    volatile unsigned long _BFAR;
    volatile unsigned long _MMAR;
	        
    stacked_r0  = stackedContextPtr[0];
    stacked_r1  = stackedContextPtr[1];
    stacked_r2  = stackedContextPtr[2];
    stacked_r3  = stackedContextPtr[3];
    stacked_r12 = stackedContextPtr[4];
    stacked_lr  = stackedContextPtr[5];
    stacked_pc  = stackedContextPtr[6];
    stacked_psr = stackedContextPtr[7];

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;   
                                                                                        
    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    //debug("[GAME OVER]\n");
    debug("R0 = 0x%008X\n", stacked_r0);
    debug("R1 = 0x%008X\n", stacked_r1);
    debug("R2 = 0x%008X\n", stacked_r2);
    debug("R3 = 0x%008X\n", stacked_r3);
    debug("R12 = 0x%08X\n", stacked_r12);
    debug("LR [R14] = 0x%08X  subroutine call return address\n", stacked_lr);
    debug("PC [R15] = 0x%08X  program counter\n", stacked_pc);
    debug("PSR = 0x%08X\n", stacked_psr);
    debug("BFAR = 0x%08X\n", (*((volatile unsigned long *)(0xE000ED38))));
    debug("CFSR = 0x%08X\n", (*((volatile unsigned long *)(0xE000ED28))));
    debug("HFSR = 0x%08X\n", (*((volatile unsigned long *)(0xE000ED2C))));
    debug("DFSR = 0x%08X\n", (*((volatile unsigned long *)(0xE000ED30))));
    debug("AFSR = 0x%08X\n", (*((volatile unsigned long *)(0xE000ED3C))));
    debug("SCB_SHCSR = 0x%08x\n", SCB->SHCSR);
    
    asm("BKPT #1");
    while (1);
}

/*!****************************************************************************
*
*/
void HardFault_Handler(void)
{
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

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

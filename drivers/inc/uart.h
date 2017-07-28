/*!****************************************************************************
* @file    		uart.h
* @author  		Storozhenko Roman - D_EL
* @version 		V1.4
* @brief   		driver for uart of STM32 F3 MCUs
* @date    		09.01.2016
* @copyright 	GNU Public License
*/
#ifndef UART_H
#define UART_H

/*!****************************************************************************
* Include
*/
#include "stm32F3xx.h"
#include "gpio.h"
#include "stdint.h"
#include "stdio.h"
#include "OSinit.h"

/*!****************************************************************************
* User define
*/
//UART1
#define     UART1_USE                   (1)
#define     UART1_Tx_HOOK               (1)
#define     UART1_Rx_HOOK               (1)
#define     UART1_TxBffSz               (128)
#define     UART1_RxBffSz               (128)
#define     UART1_RxDmaInterruptPrior   (15)
#define     UART1_TXIRQPrior            (15)
#define     UART1_PINAFTX               (7)
#define     UART1_PINAFRX               (7)
#define     UART1_HALFDUPLEX            (0)
#define     UART1_RX_IDLE_LINE_MODE     (1)

//UART2
#define     UART2_USE                   (0)
#define     UART2_Tx_HOOK               (1)
#define     UART2_Rx_HOOK               (1)
#define     UART2_TxBffSz               (128)
#define     UART2_RxBffSz               (128)
#define     UART2_RxDmaInterruptPrior   (15)
#define     UART2_TXIRQPrior            (15)
#define     UART2_PINAFTX               (7)
#define     UART2_PINAFRX               (7)
#define     UART2_HALFDUPLEX            (0)
#define     UART2_RX_IDLE_LINE_MODE     (1)

//UART3
#define     UART3_USE                   (1)
#define     UART3_Tx_HOOK               (0)
#define     UART3_Rx_HOOK               (1)
#define     UART3_TxBffSz               (128)
#define     UART3_RxBffSz               (128)
#define     UART3_RxDmaInterruptPrior   (15)
#define     UART3_TXIRQPrior            (15)
#define     UART3_PINAFTX               (7)
#define     UART3_PINAFRX               (7)
#define     UART3_HALFDUPLEX            (1)
#define     UART3_RX_IDLE_LINE_MODE     (1)

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/
typedef enum{
    BR9600,
    BR38400,
    BR115200
}uartBaudRate_type;

typedef enum{  
    uartTxFree,
    uartTxRun,
    uartTxSuccess,
    uartTxErr
}uartTxState_type;

typedef enum{  
    uartRxFree,
    uartRxRun,
    uartRxSuccess,
    uartRxStop,
    uartRxErr
}uartRxState_type;

typedef struct{
    USART_TypeDef           	*pUart;
    uint8_t                 	*pTxBff;
    uint8_t                 	*pRxBff;
    DMA_Channel_TypeDef     	*pUartTxDmaCh;
    DMA_Channel_TypeDef     	*pUartRxDmaCh;
    volatile uartTxState_type 	txState     	:8;
    volatile uartRxState_type  	rxState     	:8;
    uartTxState_type       	    baudRate    	:4;
    uint8_t                 	halfDuplex  	:1;
    uint8_t						rxIdleLineMode	:1;
    volatile uint16_t         	txCnt;
    volatile uint16_t          	rxCnt;
    volatile uint16_t           errorRxCnt;
}uart_type;

/*!****************************************************************************
* Extern viriables
*/
extern uint32_t usartBaudRate[3];
#if (UART1_USE == 1)
extern uart_type            *uart1;
#endif //UART1_USE

#if (UART2_USE == 1)
extern uart_type            *uart2;
#endif //UART2_USE

#if (UART3_USE == 1)
extern uart_type            *uart3;
#endif //UART3_USE

/*!****************************************************************************
* Macro functions
*/
#define uartGetRemainTx(uartx)      (uartx->pUartTxDmaCh->CNDTR)
#define uartGetRemainRx(uartx)      (uartx->pUartRxDmaCh->CNDTR)

__attribute__((always_inline)) __STATIC_INLINE
void uart1TxHook(){
    BaseType_t  xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart1Sem, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken != pdFALSE){
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}

__attribute__((always_inline)) __STATIC_INLINE
void uart1RxHook(){
	uart1TxHook();
}

__attribute__((always_inline)) __STATIC_INLINE
void uart3RxHook(){
    BaseType_t  xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uart3Sem, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken != pdFALSE){
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}

/*!****************************************************************************
* Prototypes for the functions
*/
void uart_init(uart_type *uartx, uartBaudRate_type baudRate);
void uart_deinit(uart_type *uartx);
void uart_setBaud(uart_type *uartx, uartBaudRate_type baudRate);
void uart_write(uart_type *uartx, void *src, uint16_t len);
void uart_read(uart_type *uartx, void *dst, uint16_t len);
void uart_stopRead(uart_type *uartStruct);

#endif //UART_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

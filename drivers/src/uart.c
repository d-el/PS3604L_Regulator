/*!****************************************************************************
 * @file		crc.с
 * @author		d_el
 * @version		V1.5
 * @date		12.12.2017
 * @brief		Driver for uart STM32F3 MCUs
 * @copyright	Copyright (C) 2017 Storozhenko Roman
 *				All rights reserved
 *				This software may be modified and distributed under the terms
 *				of the BSD license.	 See the LICENSE file for details
 */

/*!****************************************************************************
 * Include
 */
#include <stdio.h>
#include "gpio.h"
#include "OSinit.h"
#include "uart.h"

/*!****************************************************************************
 * uart1 memory
 */
#if (UART1_USE > 0)
uart_type uart1Sct;
uart_type *uart1 = &uart1Sct;
uint8_t uart1TxBff[UART1_TxBffSz];
uint8_t uart1RxBff[UART1_RxBffSz];
#endif //UART1_USE

/*!****************************************************************************
 * uart2 memory
 */
#if (UART2_USE > 0)
uart_type uart2Sct;
uart_type *uart2 = &uart2Sct;
uint8_t uart2TxBff[UART2_TxBffSz];
uint8_t uart2RxBff[UART2_RxBffSz];
#endif //UART2_USE

/*!****************************************************************************
 * uart3 memory
 */
#if (UART3_USE > 0)
uart_type uart3Sct;
uart_type *uart3 = &uart3Sct;
uint8_t uart3TxBff[UART3_TxBffSz];
uint8_t uart3RxBff[UART3_RxBffSz];
#endif //UART3_USE

#define uartMakeMantissa(baud)		(UART_FREQ / 16 / (baud))
#define uartMakeFraction(baud)		(((UART_FREQ + (baud) / 2)	/ (baud)) - (uartMakeMantissa(baud) * 16))
#define uartMakeBrr(baud)			(uartMakeMantissa(baud) << USART_BRR_DIV_MANTISSA_Pos | uartMakeFraction(baud))

uint16_t usartBaudRateDiv[] = {
	uartMakeBrr(9600),
	uartMakeBrr(38400),
	uartMakeBrr(57600),
	uartMakeBrr(115200),
};

/*!****************************************************************************
 * @brief
 */
void uart_init(uart_type *uartx, uartBaudRate_type baudRate){
	#if(UART1_USE > 0)
	if(uartx == uart1){
		/************************************************
		 * Memory setting
		 */
		uartx->pUart			= USART1;
		uartx->pTxBff			= uart1TxBff;
		uartx->pRxBff			= uart1RxBff;
		uartx->dmaMode = UART1_DMA_MODE > 0;
		uartx->rxIdleLineMode = UART1_RX_IDLE_LINE_MODE > 0;
		uartx->pUartTxDmaCh		= DMA1_Channel4;
		uartx->pUartRxDmaCh		= DMA1_Channel5;
		uartx->dmaIfcrTx		= &DMA1->IFCR;
		uartx->dmaIfcrRx		= &DMA1->IFCR;
		uartx->dmaIfcrMaskTx	= DMA_IFCR_CTCIF4;
		uartx->dmaIfcrMaskRx	= DMA_IFCR_CTCIF5;

		#if(UART1_RX_IDLE_LINE_MODE > 0)
		uartx->rxIdleLineMode = 1;
		#endif

		/************************************************
		 * IO
		 */
		gppin_init(GPIOA, 9, alternateFunctionPushPull, pullDisable, 0, UART1_PINAFTX);			//PA9, PB6 USART1_TX
		//gppin_init(GPIOA, 9, alternateFunctionOpenDrain, pullUp, 0, UART1_PINAFTX);
		#if(UART1_HALFDUPLEX == 0)
		gppin_init(GPIOA, 10, alternateFunctionPushPull, pullUp, 0, UART1_PINAFRX);				//PA10, PB7 USART1_RX
		#else
		uartx->halfDuplex = 1;
		#endif

		/************************************************
		 * NVIC
		 */
		NVIC_EnableIRQ(USART1_IRQn);
		NVIC_SetPriority(USART1_IRQn, UART1_TXIRQPrior);
		#if(UART1_RX_IDLE_LINE_MODE == 0)
		NVIC_EnableIRQ(DMA1_Channel5_IRQn);
		NVIC_SetPriority(DMA1_Channel5_IRQn, UART1_RxDmaInterruptPrior);
		#endif

		/************************************************
		 * USART clock
		 */
		RCC->CFGR3 |= RCC_CFGR3_USART1SW_SYSCLK;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;

		/************************************************
		 * DMA clock
		 */
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	}
	#endif //UART1_USE

	#if(UART2_USE > 0)
	if(uartx == uart2){
		/************************************************
		 * Memory setting
		 */
		uartx->pUart			= USART2;
		uartx->pTxBff			= uart2TxBff;
		uartx->pRxBff			= uart2RxBff;
		uartx->dmaMode = UART2_DMA_MODE > 0;
		uartx->rxIdleLineMode = UART2_RX_IDLE_LINE_MODE > 0;
		#if(UART2_DMA_MODE > 0)
		uartx->pUartTxDmaCh		= DMA1_Channel7;
		uartx->pUartRxDmaCh		= DMA1_Channel6;
		uartx->dmaIfcrTx		= &DMA1->IFCR;
		uartx->dmaIfcrRx		= &DMA1->IFCR;
		uartx->dmaIfcrMaskTx	= DMA_IFCR_CTCIF7;
		uartx->dmaIfcrMaskRx	= DMA_IFCR_CTCIF6;
		#endif

		/************************************************
		 * IO
		 */
		//gppin_init(GPIOB, 3, alternateFunctionPushPull, pullDisable, 0, UART2_PINAFTX);			//PA2 USART2_TX
		gppin_init(GPIOB, 3, alternateFunctionOpenDrain, pullDisable, 0, UART2_PINAFTX);	//PD8 USART3_TX
		#if(UART2_HALFDUPLEX == 0)
		gppin_init(GPIOB, 4, alternateFunctionPushPull, pullUp, 0, UART2_PINAFRX);				//PA3 USART2_RX
		#else
		uartx->halfDuplex = 1;
		#endif

		/************************************************
		 * NVIC
		 */
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_SetPriority(USART2_IRQn, UART2_TXIRQPrior);
		#if (UART2_RX_IDLE_LINE_MODE == 0 && UART2_DMA_MODE > 0)
		NVIC_EnableIRQ(DMA1_Channel6_IRQn);
		NVIC_SetPriority(DMA1_Channel6_IRQn, UART2_RxDmaInterruptPrior);
		#endif

		/************************************************
		 * USART clock
		 */
		RCC->CFGR3 |= RCC_CFGR3_USART2SW_SYSCLK;
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;

		/************************************************
		 * DMA clock
		 */
		#if(UART2_DMA_MODE > 0)
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		#endif
	}
		#endif //UART2_USE

	#if(UART3_USE > 0)
	if(uartx == uart3){
		/************************************************
		 * Memory setting
		 */
		uartx->pUart			= USART3;
		uartx->pTxBff			= uart3TxBff;
		uartx->pRxBff			= uart3RxBff;
		uartx->dmaMode = UART3_DMA_MODE > 0;
		uartx->rxIdleLineMode = UART3_RX_IDLE_LINE_MODE > 0;
		uartx->pUartTxDmaCh		= DMA1_Channel2;
		uartx->pUartRxDmaCh		= DMA1_Channel3;
		uartx->dmaIfcrTx		= &DMA1->IFCR;
		uartx->dmaIfcrRx		= &DMA1->IFCR;
		uartx->dmaIfcrMaskTx	= DMA_IFCR_CTCIF2;
		uartx->dmaIfcrMaskRx	= DMA_IFCR_CTCIF3;

		#if (UART3_RX_IDLE_LINE_MODE > 0)
		uartx->rxIdleLineMode = 1;
		#endif

		/************************************************
		 * IO
		 */
		//gppin_init(GPIOD, 8, alternateFunctionPushPull, pullDisable, 0, UART3_PINAFTX);	//PD8 USART3_TX
		gppin_init(GPIOD, 8, alternateFunctionOpenDrain, pullDisable, 0, UART3_PINAFTX);	//PD8 USART3_TX
		#if (UART3_HALFDUPLEX == 0)
		gppin_init(GPIOD, 9, alternateFunctionPushPull, pullUp, 0, UART3_PINAFRX);			//PD9 USART3_RX
		#else
		uartx->halfDuplex = 1;
		#endif

		/************************************************
		 * NVIC
		 */
		NVIC_EnableIRQ(USART3_IRQn);
		NVIC_SetPriority(USART3_IRQn, UART3_TXIRQPrior);
		#if(UART3_RX_IDLE_LINE_MODE == 0)
		NVIC_EnableIRQ(DMA1_Channel3_IRQn);
		NVIC_SetPriority(DMA1_Channel3_IRQn, UART3_RxDmaInterruptPrior);
		#endif

		/************************************************
		 * USART clock
		 */
		RCC->CFGR3 |= RCC_CFGR3_USART3SW_SYSCLK;
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;

		/************************************************
		 * DMA clock
		 */
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	}
	#endif //UART3_USE

	/************************************************
	 * USART
	 */
	if(uartx->halfDuplex != 0){
		uartx->pUart->CR3 |= USART_CR3_HDSEL;								//Half duplex mode is selected
	}
	uartx->pUart->CR1 |= USART_CR1_UE;										//UART enable
	uartx->pUart->CR1 &= ~USART_CR1_M;										//8bit
	uartx->pUart->CR2 &= ~USART_CR2_STOP;									//1 stop bit

	uartx->pUart->BRR = usartBaudRateDiv[baudRate];							//Baud rate
	if(uartx->dmaMode > 0){
		uartx->pUart->CR3 |= USART_CR3_DMAT;								//DMA enable transmitter
		uartx->pUart->CR3 |= USART_CR3_DMAR;								//DMA enable receiver
	}
	uartx->pUart->CR1 |= USART_CR1_TE;										//Transmitter enable
	uartx->pUart->CR1 |= USART_CR1_RE;										//Receiver enable
	uartx->pUart->ICR = USART_ICR_TCCF | USART_ICR_IDLECF;					//Clear the flags
	if(uartx->rxIdleLineMode != 0){
		uartx->pUart->ICR = USART_ICR_IDLECF;								//Clear flag
		uartx->pUart->CR1 |= USART_CR1_IDLEIE;
	}
	if(uartx->dmaMode == 0){
		uartx->pUart->RQR = USART_RQR_RXFRQ /*| USART_RQR_TXFRQ*/;
		uartx->pUart->CR1 |= USART_CR1_RXNEIE;
//		uartx->pUart->CR1 |= USART_CR1_TXEIE;
	}
	uartx->pUart->CR1 |= USART_CR1_TCIE;									//Enable the interrupt transfer complete

	/************************************************
	 * DMA
	 */
	if(uartx->dmaMode > 0){
		//DMA Channel USART TX
		uartx->pUartTxDmaCh->CCR = 0;
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
		uartx->pUartTxDmaCh->CCR |= DMA_CCR_PL_0;									//Channel priority level - Medium
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_MSIZE;									//Memory size - 8 bit
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_PSIZE;									//Peripheral size - 8 bit
		uartx->pUartTxDmaCh->CCR |= DMA_CCR_MINC;									//Memory increment mode enabled
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_PINC;									//Peripheral increment mode disabled
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_CIRC;									//Circular mode disabled
		uartx->pUartTxDmaCh->CCR |= DMA_CCR_DIR;									//Read from memory
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_TCIE;									//Transfer complete interrupt disable
		uartx->pUartTxDmaCh->CNDTR = 0;												//Number of data
		uartx->pUartTxDmaCh->CPAR = (uint32_t) &(uartx->pUart->TDR);				//Peripheral address
		uartx->pUartTxDmaCh->CMAR = (uint32_t) NULL;									//Memory address

		//DMA Channel USART RX
		uartx->pUartRxDmaCh->CCR = 0;
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
		uartx->pUartRxDmaCh->CCR |= DMA_CCR_PL_0;									//Channel priority level - Medium
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_MSIZE;									//Memory size - 8 bit
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_PSIZE;									//Peripheral size - 8 bit
		uartx->pUartRxDmaCh->CCR |= DMA_CCR_MINC;									//Memory increment mode enabled
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_PINC;									//Peripheral increment mode disabled
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_CIRC;									//Circular mode disabled
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_DIR;									//Read from peripheral
		uartx->pUartRxDmaCh->CCR |= DMA_CCR_TCIE;									//Transfer complete interrupt enable
		uartx->pUartRxDmaCh->CNDTR = 0;												//Number of data
		uartx->pUartRxDmaCh->CPAR = (uint32_t) &(uartx->pUart->RDR);				//Peripheral address
		uartx->pUartRxDmaCh->CMAR = (uint32_t) NULL;								//Memory address
	}
}

/*!****************************************************************************
 * @brief
 */
void uart_deinit(uart_type *uartx){
	uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	#if (UART1_USE > 0)
	if(uartx->pUart == USART1){
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;								//USART1 clock disable
		NVIC_DisableIRQ(DMA1_Channel4_IRQn);
		NVIC_DisableIRQ(DMA1_Channel5_IRQn);
	}
	#endif //UART1_USE

	#if (UART2_USE > 0)
	if(uartx->pUart == USART2){
		RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;								//USART1 clock disable
		NVIC_DisableIRQ(DMA1_Channel6_IRQn);
		NVIC_DisableIRQ(DMA1_Channel7_IRQn);
	}
	#endif //UART2_USE

	#if (UART3_USE > 0)
	if(uartx->pUart == USART3){
		RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;								//USART1 clock disable
		NVIC_DisableIRQ(DMA1_Channel2_IRQn);
		NVIC_DisableIRQ(DMA1_Channel3_IRQn);
	}
	#endif //UART3_USE
}

/*!****************************************************************************
 * @brief	 transfer data buffer
 */
void uart_setBaud(uart_type *uartx, uartBaudRate_type baudRate){
	if((uartx->baudRate != baudRate)&&(baudRate < BR_NUMBER)){
		uartx->pUart->BRR = usartBaudRateDiv[baudRate];
		uartx->baudRate = baudRate;
	}
}

/*!****************************************************************************
 * @brief	Set callback uart
 */
void uart_setCallback(uart_type *uartx, uartCallback_type txHoock, uartCallback_type rxHoock){
	uartx->txHoock = txHoock;
	uartx->rxHoock = rxHoock;
}

/*!****************************************************************************
 * @brief
 */
void uart_write(uart_type *uartx, void *src, uint16_t len){
	uartx->pUart->ICR = USART_ICR_TCCF;

	if(uartx->dmaMode > 0){
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
		uartx->pUartTxDmaCh->CMAR = (uint32_t) src;									//Memory address
		uartx->pUartTxDmaCh->CNDTR = len;											//Number of data
		uartx->pUartTxDmaCh->CCR |= DMA_CCR_EN;										//Channel enabled
	}
	else{
		uartx->pCurrentTx = src;
		uartx->pEndTx = uartx->pCurrentTx + len;
		uartx->pUart->CR1 |= USART_CR1_TXEIE;
		//uartx->pUart->TDR = *uartx->pCurrentTx++;
	}
	uartx->txState = uartTxRun;
}

/******************************************************************************
 * @brief
 */
void uart_read(uart_type *uartx, void *dst, uint16_t len){
//	uartx->pUart->ICR = 0xFFFFFFFFU;											//Clear all flags
//	(void) uartx->pUart->RDR;
	(void) uartx->pUart->RDR;
	uartx->pUart->RQR = USART_RQR_RXFRQ;

	if(uartx->dmaMode > 0){
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
		uartx->pUartRxDmaCh->CMAR = (uint32_t) dst;									//Memory address
		uartx->pUartRxDmaCh->CNDTR = len;											//Number of data.
		uartx->pUartRxDmaCh->CCR |= DMA_CCR_EN;										//Channel enabled
	}
	else{
		uartx->pCurrentRx = dst;
		uartx->pEndRx = uartx->pCurrentRx + len;
	}
	uartx->rxState = uartRxRun;
}

/******************************************************************************
 * @brief
 */
void uart_stopRead(uart_type *uartx){
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;									//Channel disabled
	uartx->rxState = uartRxStop;
}

/******************************************************************************
 * Transfer complete interrupt (USART TX and IDLE RX)
 */
void USART_IRQHandler(uart_type *uartx){
	uint16_t uartsr = uartx->pUart->ISR;

	/************************************************
	 * USART TRANSFER COMPLETE
	 */
	if((uartsr & USART_ISR_TC) != 0){
		uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;								//Channel disabled
		uartx->txCnt++;
		uartx->txState = uartTxSuccess;
		if(uartx->txHoock != NULL){
			uartx->txHoock(uartx);
		}
		*uartx->dmaIfcrTx = uartx->dmaIfcrMaskTx;								//Clear flag
		uartx->pUart->ICR |= USART_ICR_TCCF;
	}
	/************************************************
	 * USART IDLE LINE interrupt
	 */
	else if((uartsr & USART_ISR_IDLE) != 0){
		uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;								//Channel disabled
		uartx->rxCnt++;
		uartx->rxState = uartRxSuccess;
		if(uartx->rxHoock != NULL){
			uartx->rxHoock(uartx);
		}
		*uartx->dmaIfcrRx = uartx->dmaIfcrMaskRx;								//Clear flag
		//Clear IDLE flag by sequence (read USART_SR register followed by a read to the USART_DR register)
		uartx->pUart->ICR = USART_ICR_IDLECF;
	}
}

/******************************************************************************
 * Transfer complete interrupt (USART TX and IDLE RX) for polling mode
 */
void USART_IRQHandlerPolling(uart_type *uartx){
	uint16_t uartsr = uartx->pUart->ISR;

	/************************************************
	 * USART Transmit data register empty
	 */
	if((uartsr & USART_ISR_TXE) != 0){
		if(uartx->pCurrentTx < uartx->pEndTx){
			uartx->pUart->TDR = *uartx->pCurrentTx++;
		}else{
			//uartx->pUart->RQR = USART_RQR_TXFRQ;	//Clear TXE
			uartx->pUart->CR1 &= ~USART_CR1_TXEIE;
		}
	}

	/************************************************
	 * USART Transmission complete
	 */
	if((uartsr & USART_ISR_TC) != 0){
		if(uartx->pCurrentTx >= uartx->pEndTx){
			uartx->txCnt++;
			uartx->txState = uartTxSuccess;
			if(uartx->txHoock != NULL){
				uartx->txHoock(uartx);
			}
			uartx->pUart->ICR = USART_ICR_TCCF;	//Clear TC
		}
	}
	/************************************************
	 * USART Read data register not empty
	 */
	else if((uartsr & USART_ISR_RXNE) != 0){
		if(uartx->pCurrentRx < uartx->pEndRx){
			*uartx->pCurrentRx++ = uartx->pUart->RDR;

			if(uartx->pCurrentRx >= uartx->pEndRx){
				uartx->rxCnt++;
				uartx->rxState = uartRxSuccess;
				if(uartx->rxHoock != NULL){
					uartx->rxHoock(uartx);
				}
//
			}
		}
		else{
			uartx->pUart->RQR = USART_RQR_RXFRQ;
		}
	}
}

/******************************************************************************
 * Transfer complete interrupt (USART RX)
 */
void DmaStreamRxIRQHandler(uart_type *uartx){
	uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;											//Channel disabled
	uartx->rxCnt++;
	uartx->rxState = uartRxSuccess;
	if(uartx->rxHoock != NULL){
		uartx->rxHoock(uartx);
	}
	*uartx->dmaIfcrRx = uartx->dmaIfcrMaskRx;											//Clear flag
}

/******************************************************************************
 * Transfer complete interrupt USART1_IRQn (USART1 TX and IDLE RX)
 */
#if (UART1_USE > 0)
void USART1_IRQHandler(void){
	USART_IRQHandler(uart1);
}

/******************************************************************************
 * Transfer complete interrupt DMA1_Channel5 (USART1 RX)
 */
#if (UART1_RX_IDLE_LINE_MODE == 0)
void DMA1_Channel5_IRQHandler(void){
	DmaStreamRxIRQHandler(uart1);									//Clear flag
}
#endif //(UART1_RX_IDLE_LINE_MODE == 0)
#endif //UART1_USE

/******************************************************************************
 * Transfer complete interrupt USART2_IRQn (USART2 TX and IDLE RX)
 */
#if (UART2_USE > 0)
void USART2_IRQHandler(void){
	#if(UART2_DMA_MODE > 0)
	USART_IRQHandler(uart2);
	#else
	USART_IRQHandlerPolling(uart2);
	#endif
}
/******************************************************************************
 * Transfer complete interrupt DMA1_Channel6 (USART2 RX)
 */
#if (UART2_RX_IDLE_LINE_MODE == 0 && UART2_DMA_MODE > 0)
void DMA1_Channel6_IRQHandler(void){
	DmaStreamRxIRQHandler(uart2);
}
#endif //(UART2_RX_IDLE_LINE_MODE == 0)
#endif //UART2_USE

/******************************************************************************
 * Transfer complete interrupt USART3_IRQn (USART3 TX and IDLE RX)
 */
#if (UART3_USE > 0)
void USART3_IRQHandler(void){
	USART_IRQHandler(uart3);
}
/******************************************************************************
 * Transfer complete interrupt DMA1_Channel3 (USART3 RX)
 */
#if (UART3_RX_IDLE_LINE_MODE == 0)
void DMA1_Channel3_IRQHandler(void){
	DmaStreamRxIRQHandler(uart3);
}
#endif //(UART2_RX_IDLE_LINE_MODE == 0)
#endif //UART3_USE

/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/

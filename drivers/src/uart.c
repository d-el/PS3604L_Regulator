/*!****************************************************************************
* @file    		uart.c
* @author  		Storozhenko Roman - D_EL
* @version 		V1.4
* @brief   		driver for uart of STM32 F3 MCUs
* @date    		09.01.2016
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "uart.h"

/*!****************************************************************************
* uart1 memory
*/
#if (UART1_USE > 0)
uart_type       uart1Sct;
uart_type       *uart1 = &uart1Sct;
uint8_t         uart1TxBff[UART1_TxBffSz];
uint8_t         uart1RxBff[UART1_RxBffSz];
#endif //UART1_USE

/*!****************************************************************************
* uart2 memory
*/
#if (UART2_USE > 0)
uart_type       uart2Sct;
uart_type       *uart2 = &uart2Sct;
uint8_t         uart2TxBff[UART2_TxBffSz];
uint8_t         uart2RxBff[UART2_RxBffSz];
#endif //UART2_USE

/*!****************************************************************************
* uart3 memory
*/
#if (UART3_USE > 0)
uart_type       uart3Sct;
uart_type       *uart3 = &uart3Sct;
volatile uint8_t         uart3TxBff[UART3_TxBffSz];
volatile uint8_t         uart3RxBff[UART3_RxBffSz];
#endif //UART3_USE

uint16_t usartBaudRateDiv[3] = {    //24MHz
    0x09c4,                 //9600
    0x0271,                 //38400
    0x00D0,                 //115200
    //Добавить другие частоты
};

//uint16_t usartBaudRateDiv[3] = {    //16MHz
//    0x0682,                 //9600
//    0x01A0,                 //38400
//    0x008A,                 //115200
//    //Добавить другие частоты
//};

uint32_t usartBaudRate[3] = {
    9600,
    38400,
    115200,
    //Добавить другие частоты
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
    	uartx->pUart        = USART1;
    	uartx->pTxBff       = uart1TxBff;
    	uartx->pRxBff       = uart1RxBff;
    	uartx->pUartTxDmaCh = DMA1_Channel4;
    	uartx->pUartRxDmaCh = DMA1_Channel5;
		#if(UART1_RX_IDLE_LINE_MODE > 0)
		uartx->rxIdleLineMode = 1;
		#endif

        /************************************************
        * IO
        */
        gppin_init(GPIOA, 9, alternateFunctionPushPull, pullDisable, 0, UART1_PINAFTX);  		//PA9, PB6 USART1_TX
        //gppin_init(GPIOA, 9, alternateFunctionOpenDrain, pullUp, 0, UART1_PINAFTX);
        #if(UART1_HALFDUPLEX == 0)
		gppin_init(GPIOA, 10, alternateFunctionPushPull, pullUp, 0, UART1_PINAFRX); 			//PA10, PB7 USART1_RX
		#else
		uartx->halfDuplex = 1;
		#endif

        /************************************************
        * NVIC
        */
        NVIC_EnableIRQ(USART1_IRQn);
        NVIC_SetPriority(USART1_IRQn, UART1_TXIRQPrior);
        #if(UART1_RX_IDLE_LINE_MODE == 0)
            NVIC_EnableIRQ(DMA1_Channel5_IRQn);                                 //Включить прерывания от DMA1_Channel 5
            NVIC_SetPriority(DMA1_Channel5_IRQn, UART1_RxDmaInterruptPrior);	//Установить приоритет
        #endif

        /************************************************
        * USART clock
        */
        RCC->CFGR3		|= RCC_CFGR3_USART1SW_SYSCLK;							//System clock (SYSCLK) selected
        RCC->APB2ENR    |= RCC_APB2ENR_USART1EN;                            	//USART1 clock enable
        RCC->APB2RSTR   |= RCC_APB2RSTR_USART1RST;                          	//USART1 reset
        RCC->APB2RSTR   &= ~RCC_APB2RSTR_USART1RST;

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
    	uartx->pUart        = USART2;
    	uartx->pTxBff       = uart2TxBff;
    	uartx->pRxBff       = uart2RxBff;
    	uartx->pUartTxDmaCh = DMA1_Channel7;
    	uartx->pUartRxDmaCh = DMA1_Channel6;
		#if (UART2_RX_IDLE_LINE_MODE > 0)
		uartx->rxIdleLineMode = 1;
		#endif

        /************************************************
        * IO
        */
        gppin_init(GPIOB, 3, alternateFunctionPushPull, pullDisable, 0, UART2_PINAFTX);			//PA2 USART2_TX
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
        #if (UART2_RX_IDLE_LINE_MODE == 0)
		NVIC_EnableIRQ(DMA1_Channel6_IRQn);                                 	//Включить прерывания от DMA1_Channel 6
		NVIC_SetPriority(DMA1_Channel6_IRQn, UART2_RxDmaInterruptPrior);    	//Установить приоритет
        #endif

        /************************************************
        * USART clock
        */
		RCC->CFGR3		|= RCC_CFGR3_USART2SW_SYSCLK;							//System clock (SYSCLK) selected
        RCC->APB1ENR 	|= RCC_APB1ENR_USART2EN;                            	//USART1 clock enable
        RCC->APB1RSTR   |= RCC_APB1RSTR_USART2RST;                          	//USART1 reset
        RCC->APB1RSTR   &= ~RCC_APB1RSTR_USART2RST;

        /************************************************
        * DMA clock
        */
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    }
    #endif //UART2_USE

    #if(UART3_USE > 0)
    if(uartx == uart3){
        /************************************************
        * Memory setting
        */
    	uartx->pUart        = USART3;
    	uartx->pTxBff       = (uint8_t*)uart3TxBff;
    	uartx->pRxBff       = (uint8_t*)uart3RxBff;
    	uartx->pUartTxDmaCh = DMA1_Channel2;
    	uartx->pUartRxDmaCh = DMA1_Channel3;
		#if (UART3_RX_IDLE_LINE_MODE > 0)
		uartx->rxIdleLineMode = 1;
		#endif

        /************************************************
        * IO
        */
        //gppin_init(GPIOD, 8, alternateFunctionPushPull, pullDisable, 0, UART3_PINAFTX);  	//PD8 USART3_TX
		gppin_init(GPIOD, 8, alternateFunctionOpenDrain, pullDisable, 0, UART3_PINAFTX);  	//PD8 USART3_TX
		#if (UART3_HALFDUPLEX == 0)
        gppin_init(GPIOD, 9, alternateFunctionPushPull, pullUp, 0, UART3_PINAFRX);  		//PD9 USART3_RX
		#else
		uartx->halfDuplex = 1;
		#endif

        /************************************************
        * NVIC
        */
        NVIC_EnableIRQ(USART3_IRQn);
        NVIC_SetPriority(USART3_IRQn, UART3_TXIRQPrior);
        #if(UART3_RX_IDLE_LINE_MODE == 0)
		NVIC_EnableIRQ(DMA1_Channel3_IRQn);                                 	//Включить прерывания от DMA1_Channel 3
		NVIC_SetPriority(DMA1_Channel3_IRQn, UART3_RxDmaInterruptPrior);    	//Установить приоритет
        #endif

        /************************************************
        * USART clock
        */
		RCC->CFGR3		|= RCC_CFGR3_USART3SW_SYSCLK;							//System clock (SYSCLK) selected
        RCC->APB1ENR 	|= RCC_APB1ENR_USART3EN;                            	//USART3 clock enable
        RCC->APB1RSTR 	|= RCC_APB1RSTR_USART3RST;                          	//USART3 reset
        RCC->APB1RSTR	&= ~RCC_APB1RSTR_USART3RST;

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
		uartx->pUart->CR3 |= USART_CR3_HDSEL;                           		//Half duplex mode is selected
	}
    uartx->pUart->CR1      |= USART_CR1_UE;                                		//UART enable
    uartx->pUart->CR1      &= ~USART_CR1_M;                                		//8bit
    uartx->pUart->CR2      &= ~USART_CR2_STOP;                             		//1 stop bit

    uartx->pUart->BRR      = usartBaudRateDiv[baudRate];                   		//Baud rate
    uartx->pUart->CR3      |= USART_CR3_DMAT;                              		//DMA enable transmitter
    uartx->pUart->CR3      |= USART_CR3_DMAR;                              		//DMA enable receiver

    uartx->pUart->CR1      |= USART_CR1_TE;                                		//Transmitter enable
    uartx->pUart->CR1      |= USART_CR1_RE;                                		//Receiver enable
    uartx->pUart->ICR      = USART_ICR_TCCF | USART_ICR_IDLECF;            		//Clear the flags
    if(uartx->rxIdleLineMode != 0){
    	uartx->pUart->ICR    = USART_ICR_IDLECF;								//Clear flag
    	uartx->pUart->CR1   |= USART_CR1_IDLEIE;
    }

    uartx->pUart->CR1      |= USART_CR1_TCIE;                              		//Enable the interrupt transfer complete

    /************************************************
    * DMA
    */
    //DMA Channel USART TX
    uartx->pUartTxDmaCh->CCR = 0;
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;                               		//Channel disabled
    uartx->pUartTxDmaCh->CCR |= DMA_CCR_PL_0;                              		//Channel priority level - Medium
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_MSIZE;                            		//Memory size - 8 bit
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_PSIZE;                          		//Peripheral size - 8 bit
    uartx->pUartTxDmaCh->CCR |= DMA_CCR_MINC;                              		//Memory increment mode enabled
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_PINC;                             		//Peripheral increment mode disabled
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_CIRC;                             		//Circular mode disabled
    uartx->pUartTxDmaCh->CCR |= DMA_CCR_DIR;                               		//Read from memory
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_TCIE;                    				//Transfer complete interrupt disable
    uartx->pUartTxDmaCh->CNDTR = 0;                                        		//Number of data
    uartx->pUartTxDmaCh->CPAR = (uint32_t)&(uartx->pUart->TDR);       			//Peripheral address
    uartx->pUartTxDmaCh->CMAR = (uint32_t)NULL;                            		//Memory address

    //DMA Channel USART RX
    uartx->pUartRxDmaCh->CCR = 0;
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;                               		//Channel disabled
    uartx->pUartRxDmaCh->CCR |= DMA_CCR_PL_0;                              		//Channel priority level - Medium
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_MSIZE;                            		//Memory size - 8 bit
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_PSIZE;                           		//Peripheral size - 8 bit
    uartx->pUartRxDmaCh->CCR |= DMA_CCR_MINC;                              		//Memory increment mode enabled
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_PINC;                             		//Peripheral increment mode disabled
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_CIRC;                             		//Circular mode disabled
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_DIR;                              		//Read from peripheral
    uartx->pUartRxDmaCh->CCR |= DMA_CCR_TCIE;                              		//Transfer complete interrupt enable
    uartx->pUartRxDmaCh->CNDTR = 0;                                        		//Number of data
    uartx->pUartRxDmaCh->CPAR = (uint32_t)&(uartx->pUart->RDR);       			//Peripheral address
    uartx->pUartRxDmaCh->CMAR = (uint32_t)NULL;                      			//Memory address
}

/*!****************************************************************************
* @brief
*/
void uart_deinit(uart_type *uartx){
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;               						//Channel disabled
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;               						//Channel disabled
    #if (UART1_USE > 0)
    if(uartx->pUart == USART1){
        RCC->APB2ENR    &= ~RCC_APB2ENR_USART1EN;           					//USART1 clock disable
        NVIC_DisableIRQ(DMA1_Channel4_IRQn);
        NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    }
    #endif //UART1_USE

    #if (UART2_USE > 0)
    if(uartx->pUart == USART2){
        RCC->APB1ENR    &= ~RCC_APB1ENR_USART2EN;           					//USART1 clock disable
        NVIC_DisableIRQ(DMA1_Channel6_IRQn);
        NVIC_DisableIRQ(DMA1_Channel7_IRQn);
    }
    #endif //UART2_USE

    #if (UART3_USE > 0)
    if(uartx->pUart == USART3){
        RCC->APB1ENR    &= ~RCC_APB1ENR_USART3EN;       						//USART1 clock disable
        NVIC_DisableIRQ(DMA1_Channel2_IRQn);
        NVIC_DisableIRQ(DMA1_Channel3_IRQn);
    }
    #endif //UART3_USE
}

/*!****************************************************************************
* @brief    transfer data buffer
*/
void uart_setBaud(uart_type *uartx, uartBaudRate_type baudRate){
    if(uartx->baudRate != baudRate){
        uartx->pUart->BRR   = usartBaudRateDiv[baudRate];
        uartx->baudRate     = baudRate;
    }
}

/*!****************************************************************************
* @brief
*/
void uart_write(uart_type *uartx, void *src, uint16_t len){
    uartx->pUartTxDmaCh->CCR &= ~DMA_CCR_EN;                               		//Channel disabled
    uartx->pUartTxDmaCh->CMAR = (uint32_t)src;                             		//Memory address
    uartx->pUartTxDmaCh->CNDTR = len;                                      		//Number of data
    uartx->pUartTxDmaCh->CCR |= DMA_CCR_EN;                                		//Channel enabled
    uartx->txState = uartTxRun;
}

/******************************************************************************
* @brief
*/
void uart_read(uart_type *uartx, void *dst, uint16_t len){
    uartx->pUart->ICR = 0xFFFFFFFFU;                                       		//Clear all flags
    (void)uartx->pUart->RDR;
    uartx->pUart->ICR = 0xFFFFFFFFU;
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;                               		//Channel disabled
    uartx->pUartRxDmaCh->CMAR = (uint32_t)dst;                             		//Memory address
    uartx->pUartRxDmaCh->CNDTR = len;                                      		//Number of data.
    uartx->pUartRxDmaCh->CCR |= DMA_CCR_EN;                                		//Channel enabled
    uartx->rxState = uartRxRun;
}

/******************************************************************************
* @brief
*/
void uart_stopRead(uart_type *uartx){
    uartx->pUartRxDmaCh->CCR &= ~DMA_CCR_EN;                               		//Channel disabled
    uartx->rxState = uartRxStop;
}

/******************************************************************************
* Transfer complete interrupt USART1_IRQn (USART1 TX and IDLE RX)
*/
#if (UART1_USE > 0)
void USART1_IRQHandler(void){
    uint16_t uartsr = uart1->pUart->ISR;

    /************************************************
	* USART TRANSFER COMPLETE
	*/
    if((uartsr & USART_ISR_TC) != 0){
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;                                     	//Channel disabled
        uart1->txCnt++;
        uart1->txState  = uartTxSuccess;
        #if (UART1_Tx_HOOK > 0)
        uart1TxHook();
        #endif //UART1_Tx_HOOK
        DMA1->IFCR      = DMA_IFCR_CTCIF4;                               		//Clear flag
        uart1->pUart->ICR |= USART_ICR_TCCF;
    }
    /************************************************
    * USART IDLE LINE interrupt
    */
    #if (UART1_RX_IDLE_LINE_MODE > 0)
    if((uartsr & USART_ISR_IDLE) != 0){
    	//uart1->pUart->CR1	&= ~USART_CR1_RE;                             		//Receiver disable
        DMA1_Channel5->CCR &= ~DMA_CCR_EN;                                     	//Channel disabled
        uart1->rxCnt++;
        uart1->rxState  = uartRxSuccess;
        #if (UART1_Rx_HOOK > 0)
        uart1RxHook();
        #endif //UART1_Rx_HOOK
        DMA1->IFCR      = DMA_IFCR_CTCIF5;                                      //Clear flag
        uart1->pUart->ICR = USART_ICR_IDLECF;
    }
    #endif
}
/******************************************************************************
* Transfer complete interrupt DMA1_Channel5 (USART1 RX)
*/
#if (UART1_RX_IDLE_LINE_MODE == 0)
void DMA1_Channel5_IRQHandler(void){
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;                                     		//Channel disabled
    uart1->rxCnt++;
    uart1->rxState  = uartRxSuccess;
    #if (UART1_Rx_HOOK > 0)
    uart1RxHook();
    #endif //UART1_Rx_HOOK
    DMA1->IFCR      = DMA_IFCR_CTCIF5;                                      	//Clear flag
}
#endif //(UART1_RX_IDLE_LINE_MODE == 0)
#endif //UART1_USE


/******************************************************************************
* Transfer complete interrupt USART2_IRQn (USART2 TX and IDLE RX)
*/
#if (UART2_USE > 0)
void USART2_IRQHandler(void){
    uint16_t uartsr = uart2->pUart->ISR;

    /************************************************
	* USART TRANSFER COMPLETE
	*/
    if((uartsr & USART_ISR_TC) != 0){
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;                                     	//Channel disabled
        uart2->txCnt++;
        uart2->txState  = uartTxSuccess;
        #if (UART2_Tx_HOOK > 0)
        uart2TxHook();
        #endif //UART2_Tx_HOOK
        DMA1->IFCR      = DMA_IFCR_CTCIF7;                                      //Clear flag
        uart2->pUart->ICR |= USART_ICR_TCCF;
    }
    /************************************************
    * USART IDLE LINE interrupt
    */
    #if (UART2_RX_IDLE_LINE_MODE > 0)
    if((uartsr & USART_ISR_IDLE) != 0){
        DMA1_Channel6->CCR &= ~DMA_CCR_EN;                                     //Channel disabled
        uart2->rxCnt++;
        uart2->rxState  = uartRxSuccess;
        #if (UART2_Rx_HOOK > 0)
        uart2RxHook();
        #endif //UART2_Rx_HOOK
        DMA1->IFCR      = DMA_IFCR_CTCIF6;                                      //Clear flag
        uart2->pUart->ICR = USART_ICR_IDLECF;
    }
    #endif
}
/******************************************************************************
* Transfer complete interrupt DMA1_Channel6 (USART2 RX)
*/
#if (UART2_RX_IDLE_LINE_MODE == 0)
void DMA1_Channel6_IRQHandler(void){
    DMA1_Channel6->CCR &= ~DMA_CCR_EN;                                     		//Channel disabled
    uart2->rxCnt++;
    uart2->rxState  = uartRxSuccess;
    #if (UART2_Rx_HOOK > 0)
    uart2RxHook();
    #endif //UART2_Rx_HOOK
    DMA1->IFCR      = DMA_IFCR_CTCIF6;                                      	//Clear flag
}
#endif //(UART2_RX_IDLE_LINE_MODE == 0)
#endif //UART2_USE

/******************************************************************************
* Transfer complete interrupt USART3_IRQn (USART3 TX and IDLE RX)
*/
#if (UART3_USE > 0)
void USART3_IRQHandler(void){
    uint16_t uartsr = uart3->pUart->ISR;

    /************************************************
	* USART TRANSFER COMPLETE
	*/
    if((uartsr & USART_ISR_TC) != 0){
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;                           			//Channel disabled
        uart3->txCnt++;
        uart3->txState  = uartTxSuccess;
        #if (UART3_Tx_HOOK > 0)
        uart3TxHook();
        #endif //UART3_Tx_HOOK
        DMA1->IFCR      = DMA_IFCR_CTCIF2;                               		//Clear flag
        uart3->pUart->ICR |= USART_ICR_TCCF;
    }
    /************************************************
    * USART IDLE LINE interrupt
    */
    #if (UART3_RX_IDLE_LINE_MODE > 0)
    if((uartsr & USART_ISR_IDLE) != 0){
        DMA1_Channel3->CCR &= ~DMA_CCR_EN;                                 		//Channel disabled
        uart3->rxCnt++;
        uart3->rxState  = uartRxSuccess;
        #if (UART3_Rx_HOOK > 0)
        uart3RxHook();
        #endif //UART3_Rx_HOOK
        DMA1->IFCR      = DMA_IFCR_CTCIF3;                             			//Clear flag
        uart3->pUart->ICR = USART_ICR_IDLECF;
    }
    #endif
}
/******************************************************************************
* Transfer complete interrupt DMA1_Channel3 (USART3 RX)
*/
#if (UART3_RX_IDLE_LINE_MODE == 0)
void DMA1_Channel3_IRQHandler(void){
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;                                     		//Channel disabled
    uart3->rxCnt++;
    uart3->rxState  = uartRxSuccess;
    #if (UART3_Rx_HOOK > 0)
    uart3RxHook();
    #endif //UART3_Rx_HOOK
    DMA1->IFCR      = DMA_IFCR_CTCIF3;                                      	//Clear flag
}
#endif //(UART2_RX_IDLE_LINE_MODE == 0)
#endif //UART3_USE

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

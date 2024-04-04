/*!****************************************************************************
 * @file		spi.c
 * @author		d_el
 * @version		V1.0
 * @date		21.03.2021
 * @copyright	The MIT License (MIT). Copyright (c) 2021 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <stddef.h>
#include "spi.h"
#include "gpio.h"

/*!****************************************************************************
* spi1 memory
*/
#if (SPI1_USE > 0)
spi_type	spi1Sct;
spi_type	*spi1 = &spi1Sct;
#endif //SPI1_USE

/*!****************************************************************************
* spi2 memory
*/
#if (SPI2_USE > 0)
spi_type	spi2Sct;
spi_type	*spi2 = &spi2Sct;
#endif //SPI2_USE

/*!****************************************************************************
* spi3 memory
*/
#if (SPI3_USE > 0)
spi_type	spi3Sct;
spi_type	*spi3 = &spi3Sct;
#endif //SPI3_USE

/*!****************************************************************************
* @brief
* @param
* @retval
*/
void spi_init(spi_type *spix, spiDiv_t divClock){
	#if (SPI3_USE > 0)
	if(spix == spi3){
		/************************************************
		* Memory setting
		*/
		spix->spi           = SPI3;
		spix->pSpiTxDmaCh   = DMA2_Channel2;
		spix->pSpiRxDmaCh   = DMA2_Channel1;

		/************************************************
		* IO
		*/
		gppin_init(GPIOB, 5, alternateFunctionPushPull, pullDisable, 0, SPI3_PINAFMOSI);
		gppin_init(GPIOB, 4, alternateFunctionPushPull, pullUp, 0, SPI3_PINAFMISO);
		gppin_init(GPIOB, 3, alternateFunctionPushPull, pullDisable, 0, SPI3_PINAFCLK);

		/************************************************
		* NVIC
		*/
		NVIC_EnableIRQ(DMA2_Channel1_IRQn);
		NVIC_SetPriority(DMA2_Channel1_IRQn, SPI3_RxDmaInterruptPrior);

		/************************************************
		* SPI clock
		*/
		RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
		RCC->APB1RSTR |= RCC_APB1RSTR_SPI3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI3RST;

		/************************************************
		* DMA clock
		*/
		RCC->AHBENR |= RCC_AHBENR_DMA2EN;
	}
	#endif //SPI3_USE

	/************************************************
    * SPI
    */
	spix->spi->CR1 |= SPI_CR1_MSTR;                             // Master configuration
	spix->spi->CR1 |= SPI_CR1_SSM;                              // Software slave management enabled
	spix->spi->CR1 |= SPI_CR1_SSI;                              // Internal slave select
	spix->spi->CR1 |= SPI_CR1_CPHA;                             // Clock phase
	spix->spi->CR1 |= divClock << SPI_CR1_BR_Pos;               // BR
	spix->spi->CR2 |= SPI_CR2_FRXTH;                            // RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)
	spix->spi->CR2 |= SPI_CR2_DS_0 |
							SPI_CR2_DS_1 |
							SPI_CR2_DS_2;                       // Data size 8-bit
	spix->spi->CR2 |= SPI_CR2_TXDMAEN;                          // Tx buffer DMA enabled
	spix->spi->CR2 |= SPI_CR2_RXDMAEN;                          // Rx buffer DMA enabled


	spix->spi->CR1 |= SPI_CR1_SPE;                              //SPI enable

    /************************************************
	 * DMA
	 */
	//DMA Channel SPI TX
	spix->pSpiTxDmaCh->CCR &= ~DMA_CCR_EN;                         //Channel disabled
	spix->pSpiTxDmaCh->CCR |= DMA_CCR_PL_0;                        //Channel priority level - Medium
	spix->pSpiTxDmaCh->CCR &= ~DMA_CCR_MSIZE;                      //Memory size - 8 bit
	spix->pSpiTxDmaCh->CCR &= ~DMA_CCR_PSIZE;                      //Peripheral size - 8 bit
	spix->pSpiTxDmaCh->CCR |= DMA_CCR_MINC;                        //Memory increment mode enabled
	spix->pSpiTxDmaCh->CCR &= ~DMA_CCR_PINC;                       //Peripheral increment mode disabled
	spix->pSpiTxDmaCh->CCR &= ~DMA_CCR_CIRC;                       //Circular mode disabled
	spix->pSpiTxDmaCh->CCR |= DMA_CCR_DIR;                         //Read from memory
	spix->pSpiTxDmaCh->CCR &= ~DMA_CCR_TCIE;                       //Transfer complete interrupt disable
	spix->pSpiTxDmaCh->CNDTR = 0;                                  //Number of data
	spix->pSpiTxDmaCh->CPAR = (uint32_t) &(spix->spi->DR);         //Peripheral address
	spix->pSpiTxDmaCh->CMAR = (uint32_t) NULL;                      //Memory address
	//DMA Channel SPI RX
	spix->pSpiRxDmaCh->CCR &= ~DMA_CCR_EN;                         //Channel disabled
	spix->pSpiRxDmaCh->CCR |= DMA_CCR_PL_0;                        //Channel priority level - Medium
	spix->pSpiRxDmaCh->CCR &= ~DMA_CCR_MSIZE;                      //Memory size - 8 bit
	spix->pSpiRxDmaCh->CCR &= ~DMA_CCR_PSIZE;                      //Peripheral size - 8 bit
	spix->pSpiRxDmaCh->CCR |= DMA_CCR_MINC;                        //Memory increment mode enabled
	spix->pSpiRxDmaCh->CCR &= ~DMA_CCR_PINC;                       //Peripheral increment mode disabled
	spix->pSpiRxDmaCh->CCR &= ~DMA_CCR_CIRC;                       //Circular mode disabled
	spix->pSpiRxDmaCh->CCR &= ~DMA_CCR_DIR;                        //Read from peripheral
	spix->pSpiRxDmaCh->CCR |= DMA_CCR_TCIE;                        //Transfer complete interrupt enable
	spix->pSpiRxDmaCh->CNDTR = 0;                                  //Number of data
	spix->pSpiRxDmaCh->CPAR = (uint32_t) &(spix->spi->DR);         //Peripheral address
	spix->pSpiRxDmaCh->CMAR = (uint32_t)NULL;                      //Memory address
}

/*!****************************************************************************
* @brief    spi transfer data
*/
void spi_transfer(spi_type *spix, void *dst, void *src, uint16_t len){
	spix->pSpiTxDmaCh->CCR &= ~DMA_CCR_EN;
	spix->pSpiRxDmaCh->CCR &= ~DMA_CCR_EN;

	spix->pSpiTxDmaCh->CMAR = (uint32_t) src;
	spix->pSpiRxDmaCh->CMAR = (uint32_t) dst;

	spix->pSpiTxDmaCh->CNDTR = len;
	spix->pSpiRxDmaCh->CNDTR = len;

	spix->pSpiRxDmaCh->CCR |= DMA_CCR_EN;
	spix->pSpiTxDmaCh->CCR |= DMA_CCR_EN;

	spix->state = spiRun;
}

/******************************************************************************
*
*/
void spi_setCallback(spi_type *spix, spiCallback_type tcHoock){
	spix->tcHoock = tcHoock;
}

/*!****************************************************************************
* IRQ SPI3
*/
#if (SPI3_USE > 0)
void DMA2_Channel1_IRQHandler(void){	//RX
	if(spi3->tcHoock){
		spi3->tcHoock(spi3);
	}
	DMA2->IFCR = DMA_IFCR_CTCIF1;	//Clear flag
	spi3->state = spiTCSuccess;
}
#endif

/******************************** END OF FILE ********************************/

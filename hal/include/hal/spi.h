/*!****************************************************************************
 * @file		spi.h
 * @author		d_el
 * @version		V1.0
 * @date		21.03.2021
 * @copyright	The MIT License (MIT). Copyright (c) 2021 Storozhenko Roman
 */

#ifndef spi_H
#define spi_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
 * Include
 */
#include <stm32f3xx.h>
#include <stdint.h>

/*!****************************************************************************
 * Define
 */

//SPI3
#define		SPI3_USE					(1)
#define		SPI3_RxDmaInterruptPrior	(14)
#define		SPI3_PINAFMOSI				(6)
#define		SPI3_PINAFMISO				(6)
#define		SPI3_PINAFCLK				(6)

/*!****************************************************************************
 * Typedef
 */
typedef enum{
	spiFree,
	spiRun,
	spiTCSuccess,
	spiTCErr
}spiState_type;

typedef enum{
	spiDiv2,
	spiDiv4,
	spiDiv8,
	spiDiv16,
	spiDiv32,
	spiDiv64,
	spiDiv128,
	spiDiv256,
}spiDiv_t;

typedef struct spiStruct{
	SPI_TypeDef				*spi	;
	DMA_Channel_TypeDef		*pSpiTxDmaCh;
	DMA_Channel_TypeDef		*pSpiRxDmaCh;
	void (*tcHoock)(struct spiStruct *spix);
	uint8_t					clockDiv;
	volatile spiState_type			state	:8;
}spi_type;

typedef void (*spiCallback_type)(spi_type *spix);

/*!****************************************************************************
 * Exported variables
 */
#if (SPI1_USE > 0)
extern spi_type				*spi1;
#endif //SPI1_USE

#if (SPI2_USE > 0)
extern spi_type				*spi2;
#endif //SPI2_USE

#if (SPI3_USE > 0)
extern spi_type				*spi3;
#endif //SPI3_USE

/*!****************************************************************************
 * Function declaration
 */
void spi_init(spi_type *spix, spiDiv_t divClock);
void spi_setCallback(spi_type *spix, spiCallback_type tcHoock);
void spi_transfer(spi_type *spix, void *dst, const void *src, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif //spi_H
/******************************** END OF FILE ********************************/

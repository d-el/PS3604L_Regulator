/*!****************************************************************************
 * @file		adcTSK.c
 * @author		Storozhenko Roman - D_EL
 * @version 	V1.0.0
 * @date		07-01-2015
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
 * Include
 */
#include <array>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <adc.h>
#include <spi.h>
#include <ad5663.h>
#include "adcTSK.h"
#include <movingAverageFilter.h>
#include "sysTimeMeas.h"
#include <ad468x.h>

/*!****************************************************************************
 * MEMORY
 */
adcTaskStct_type adcTaskStct;
SemaphoreHandle_t AdcEndConversionSem;
adcStct_type adcValue;
SemaphoreHandle_t spiSem;

/*!****************************************************************************
 * @brief	SPI callback
 */
static void spiTC_Hook(spi_type *spix){
	(void)spix;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(spiSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

static bool spi(void *dst, const void *src, uint16_t len){
	spi_transfer(spi3, dst, src, len);
	BaseType_t res = xSemaphoreTake(spiSem, pdMS_TO_TICKS(portMAX_DELAY));
	return res == pdTRUE && spi3->state == spiTCSuccess;
}

static void adcHoock(adcStct_type *adc){
	adcValue = *adc;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(AdcEndConversionSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*!****************************************************************************
 * @brief
 * @param
 * @retval
 */
void adcTSK(void *pPrm){
	(void)pPrm;
	adcTaskStct_type& a = adcTaskStct;

	vSemaphoreCreateBinary(AdcEndConversionSem);
	xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

	vSemaphoreCreateBinary(spiSem);
	xSemaphoreTake(spiSem, portMAX_DELAY);
	spi_init(spi3, spiDiv4);
	spi_setCallback(spi3, spiTC_Hook);

	static MovingAverageFilter<uint16_t, 16> f_vin(45000); // 55.5V init
	static MovingAverageFilter<uint32_t, 1000> f_iadc(0);
	static MovingAverageFilter<uint32_t, 1000> f_uadc(0);

	static MovingAverageFilter<int32_t, 16> f_iex(0);
	static MovingAverageFilter<int16_t, 16> f_common(820);

	decltype(a.dacU) dacU = 0;
	decltype(a.dacI) dacI = 0;

	adc_setCallback(adcHoock);
	ad5663_init(spi);
	ad468x_init(spi);
	adc_setSampleRate(1000);
	adc_startSampling();

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);
		int32_t resa = 0, resb = 0;
		ad468x_convRead(&resa, &resb);
		a.filtered.i = f_iadc.proc(resb);
		a.filtered.u = f_uadc.proc(resa);
		a.filtered.vrefm = f_common.proc(adcValue.adcreg[CH_VREFM]);
		a.filtered.uin = f_vin.proc(adcValue.adcreg[CH_UINADC]) - a.filtered.vrefm;
		if(dacU != a.dacU){
			ad5663_set_b(a.dacU);
			dacU = a.dacU;
		}
		if(dacI != a.dacI){
			ad5663_set_a(a.dacI);
			dacI = a.dacI;
		}
	}
}

/******************************** END OF FILE ********************************/

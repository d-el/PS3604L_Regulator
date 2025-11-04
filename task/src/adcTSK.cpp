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
#include <ad5060.h>
#include "adcTSK.h"
#include <movingAverageFilter.h>
#include "sysTimeMeas.h"
#include <ad468x.h>
#include <prmSystem.h>

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

	static MovingAverageFilter<uint16_t, 16> f_tsh1(0);
	static MovingAverageFilter<uint16_t, 16> f_tsh2(0);
	static MovingAverageFilter<uint16_t, 16> f_vin(45000);
	static MovingAverageFilter<uint32_t, 1000> f_iadc(0);
	static MovingAverageFilter<uint32_t, 1000> f_vadc(0);
	static MovingAverageFilter<int16_t, 32> f_common(820);

	decltype(a.dacV) dacU = 0;
	decltype(a.dacI) dacI = 0;

	adc_setCallback(adcHoock);
	ad5060_init(spi);
	ad468x_init(spi);
	adc_setSampleRate(500);
	adc_startSampling();

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);
		int32_t resa = 0, resb = 0;
		ad468x_convRead(&resa, &resb);
		a.filtered.i = f_iadc.proc(resb);
		a.filtered.v = f_vadc.proc(resa);

		a.filtered.vrefm = f_common.proc(adcValue.adcreg[CH_VREFM]);
		a.filtered.tsh1 = f_tsh1.proc(adcValue.adcreg[CH_TSH1]) - a.filtered.vrefm;
		a.filtered.tsh2 = f_tsh2.proc(adcValue.adcreg[CH_TSH2]) - a.filtered.vrefm;
		a.filtered.vin = f_vin.proc(adcValue.adcreg[CH_UINADC]) - a.filtered.vrefm;

		// Set DAC value
		if(dacU != a.dacV){
			ad5060_set_b(a.dacV);
			dacU = a.dacV;
		}
		if(dacI != a.dacI){
			ad5060_set_a(a.dacI);
			dacI = a.dacI;
		}

		// Change filter size
		if(f_iadc.getsize() != Prm::ifilter_size.val){
			f_iadc.setsize(Prm::ifilter_size.val);
		}
		if(f_vadc.getsize() != Prm::vfilter_size.val){
			f_vadc.setsize(Prm::vfilter_size.val);
		}
	}
}

/******************************** END OF FILE ********************************/

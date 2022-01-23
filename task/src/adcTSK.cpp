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
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <adc.h>
#include <ina229.h>
#include "adcTSK.h"
#include <movingAverageFilter.h>

/*!****************************************************************************
 * MEMORY
 */
adcTaskStct_type adcTaskStct;
SemaphoreHandle_t AdcEndConversionSem;
adcStct_type adcValue;

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

	static MovingAverageFilter<uint16_t, 64> f_vin(40);
	static MovingAverageFilter<uint16_t, 256> f_iadc(0);
	static MovingAverageFilter<uint16_t, 256> f_uadc(0);
	static MovingAverageFilter<int32_t, 512> f_iex(0);

	adc_setCallback(adcHoock);
	a.externalSensorOk = ina229_init();
	adc_setSampleRate(1000);
	adc_startSampling();

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

		int32_t shunt = 0;
		if(a.externalSensorOk != 0){
			ina229_readShuntVoltage(&shunt);
			shunt = shunt / 16;
			ina229_trig();
		}

		a.filtered.uin = f_vin.proc(adcValue.adcreg[CH_UINADC]);
		a.filtered.i = f_iadc.proc(adcValue.adcreg[CH_IADC]);
		a.filtered.u = f_uadc.proc(adcValue.adcreg[CH_UADC]);
		a.filtered.iex = f_iex.proc(shunt);
	}
}

/******************************** END OF FILE ********************************/

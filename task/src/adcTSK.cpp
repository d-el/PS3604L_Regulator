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
#include "sysTimeMeas.h"

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

	static MovingAverageFilter<uint16_t, 16> f_vin(45000); // 55.5V init
	static MovingAverageFilter<uint16_t, 256> f_iadc(0);
	static MovingAverageFilter<uint16_t, 256> f_uadc(0);
	static MovingAverageFilter<int32_t, 128> f_iex(0);

	adc_setCallback(adcHoock);
	a.externalSensorOk = ina229_init();
	ina229_trig();
	adc_setSampleRate(1000);
	adc_startSampling();

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

		a.filtered.uin = f_vin.proc(adcValue.adcreg[CH_UINADC]);
		a.filtered.i = f_iadc.proc(adcValue.adcreg[CH_IADC]);
		a.filtered.u = f_uadc.proc(adcValue.adcreg[CH_UADC]);

		if(a.externalSensorOk != 0){
			bool inaConverted = false;
			ina229_readCNVRF(&inaConverted);
			if(inaConverted){
				int32_t shunt = 0;
				ina229_readShuntVoltage(&shunt);
				a.filtered.iex = f_iex.proc(shunt);
			}
		}
	}
}

/******************************** END OF FILE ********************************/

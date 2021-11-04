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
#include <specificMath.h>
#include <adc.h>
#include <ina229.h>
#include "adcTSK.h"
#include "systemTSK.h"
#include <prmSystem.h>

/*!****************************************************************************
 * Local function declaration
 */
static inline uint16_t movingAverageFilter(adcFilt_type *f, uint16_t v);
static inline void aInit(void);

/*!****************************************************************************
 * MEMORY
 */
adcTaskStct_type adcTaskStct = {
	.adcFilt = {
		[CH_UINADC] = { .adcDefVal = 40, .oversampling = 1, .recursiveK = 1, .MA_filter_WITH = 16, },
		[CH_IADC] = { .adcDefVal = 0, .oversampling = 1, .recursiveK = 1, .MA_filter_WITH = 64, },
		[CH_UADC] = { .adcDefVal = 0, .oversampling = 1, .recursiveK = 1, .MA_filter_WITH = 32, }
	}
};

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
	_iq qtemp;
	uint8_t thinningCnt = 0;

    //AdcEndConversionSem
    vSemaphoreCreateBinary(AdcEndConversionSem);
    xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

    adc_setCallback(adcHoock);
    a.externalSensorOk = ina229_init() ? 1 : 0;
	aInit();
	adc_startSampling();

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);
		for(uint8_t index = 0; index < ADC_NUM_CH; index++){
			// Oversampling
			uint32_t val = adcValue.adcreg[index] * a.adcFilt[index].oversampling;
			// Apply filter
			a.filtered[index] = movingAverageFilter(&a.adcFilt[index], val);
		}

		/*
		 */
		if(a.externalSensorOk != 0){
			if(thinningCnt == 0){
				int32_t shunt;
				ina229_readShuntVoltage(&shunt);
				a.adcIna229 = shunt >> 8;
				ina229_trig();
			}
			thinningCnt++;
			if(thinningCnt >= 8){
				thinningCnt = 0;
			}
		}

		/*
		 * Calculate current on external ADC
		 */
		if(a.adcIna229 <= Prm::iext1_adc){
			a.currentExt = s32iq_Fy_x1x2y1y2x(Prm::iext0_adc, Prm::iext1_adc,
													IntToIQ(Prm::iext0_i, 1000000), IntToIQ(Prm::iext1_i, 1000000),
													a.adcIna229);
		}
		else{
			a.currentExt = s32iq_Fy_x1x2y1y2x(Prm::iext1_adc, Prm::iext2_adc,
													IntToIQ(Prm::iext1_i, 1000000), IntToIQ(Prm::iext2_i, 1000000),
													a.adcIna229);
		}
		if(a.currentExt < 0){
			a.currentExt = 0;
		}

		/*
		 * Calculate voltage
		 */
		if(a.filtered[CH_UADC] <= Prm::v1_adc){
			a.voltage = s32iq_Fy_x1x2y1y2x(Prm::v0_adc, Prm::v1_adc,
													IntToIQ(Prm::v0_u, 1000000), IntToIQ(Prm::v1_u, 1000000),
													a.filtered[CH_UADC]);
		}
		else if(a.filtered[CH_UADC] <= Prm::v2_adc){
			a.voltage = s32iq_Fy_x1x2y1y2x(Prm::v1_adc, Prm::v2_adc,
													IntToIQ(Prm::v1_u, 1000000), IntToIQ(Prm::v2_u, 1000000),
													a.filtered[CH_UADC]);
		}
		else{
			a.voltage = s32iq_Fy_x1x2y1y2x(Prm::v2_adc, Prm::v3_adc,
													IntToIQ(Prm::v2_u, 1000000), IntToIQ(Prm::v3_u, 1000000),
													a.filtered[CH_UADC]);
		}
		if(a.voltage < 0){
			a.voltage = 0;
		}

		/*
		 * Detect revers voltage
		 */
//		if(a.adcFilt[CH_UADC].recursiveFilterOut > REVERSE_VOLTAGE_THRESHOLD){
//			a.reverseVoltage = 0;
//		}else{
//			a.reverseVoltage = 1;
//		}

		/*
		 * Calculate current
		 */
		if(a.filtered[CH_IADC] <= Prm::i1_adc){
			a.currentInt = s32iq_Fy_x1x2y1y2x(Prm::i0_adc, Prm::i1_adc,
													IntToIQ(Prm::i0_i, 1000000), IntToIQ(Prm::i1_i, 1000000),
													a.filtered[CH_IADC]);
		}
		else if(a.filtered[CH_IADC] <= Prm::i2_adc){
			a.currentInt = s32iq_Fy_x1x2y1y2x(Prm::i1_adc, Prm::i2_adc,
													IntToIQ(Prm::i1_i, 1000000), IntToIQ(Prm::i2_i, 1000000),
													a.filtered[CH_IADC]);
		}
		else{
			a.currentInt = s32iq_Fy_x1x2y1y2x(Prm::i2_adc, Prm::i3_adc,
													IntToIQ(Prm::i2_i, 1000000), IntToIQ(Prm::i3_i, 1000000),
													a.filtered[CH_IADC]);
		}
		if(a.currentInt < 0){
			a.currentInt = 0;
		}

		/*
		 * Select current sensor
		 */
		if((a.currentInt >= _IQ(CURRENT_SENSOR_THRESHOLD_UP / 1000.0f))
				|| (a.externalSensorOk == 0)){
			a.currentSensor = adcCurrentSensorInternal;
		}
		if((a.currentInt < _IQ(CURRENT_SENSOR_THRESHOLD_DOWN / 1000.0f))
				&& (a.externalSensorOk != 0)){
			a.currentSensor = adcCcurrentSensorExternal;
		}

		a.current = a.currentSensor == adcCurrentSensorInternal ? a.currentInt : a.currentExt;

		/*
		 * Calculate input voltage
		 */
		a.udc = a.filtered[CH_UINADC] * _IQ((AdcVref * (UDC_Rh + UDC_Rl)) / (65536 * UDC_Rl));

		/*
		 * Calculate output power
		 */
		if(Prm::enable){
			a.outPower = _IQ14mpy(_IQtoIQ14(a.voltage), _IQtoIQ14(a.current));
		}else{
			a.outPower = 0;
		}

		/*
		 * Calculate dissipation power
		 */
		a.radPower = _IQ14mpy(_IQtoIQ14(a.udc - a.voltage),
				_IQtoIQ14(a.current));

		/*
		 * Calculate resistens
		 */
		if(Prm::enable && (a.current > _IQ(0.001))
				&& (a.voltage > _IQ(0.05))){
			qtemp = _IQ14div(_IQtoIQ14(a.voltage), _IQtoIQ14(a.current));
			if(qtemp > _IQ14(99999)){  //limit 99999 Ohm
				qtemp = _IQ14(99999);
			}
			a.resistens = qtemp;
		}else{
			a.resistens = _IQ14(99999);
		}

		/*
		 * Calculate capacity
		 */
		static uint64_t capacity;
		if(Prm::enable){
			capacity += a.current;
			if(capacity >= ((uint64_t) _IQ(0.001) * (1000000 / adcValue.sampleRate)
							* 60 * 60)){
				a.capacity += 1;
				capacity = capacity
						- ((uint64_t) _IQ(0.001)
								* (1000000 / adcValue.sampleRate) * 60 * 60);
			}
		}else{
			a.capacity = 0;
			capacity = 0;
		}
	}
}

/*!****************************************************************************
 * @brief	Init to default adc task memory
 */
static inline void aInit(void){
	for(uint8_t ch = 0; ch < ADC_NUM_CH; ch++){
		adcTaskStct.adcFilt[ch].recursiveFilterCumul = adcTaskStct.adcFilt[ch].adcDefVal
						<< adcTaskStct.adcFilt[ch].recursiveK;
		for(uint16_t i = 0; i < adcTaskStct.adcFilt[ch].MA_filter_WITH; i++){
			adcTaskStct.adcFilt[ch].MA_filterMas[i] = adcTaskStct.adcFilt[ch].adcDefVal;
		}
	}
}

/*!****************************************************************************
 * @brief
 */
static inline uint16_t movingAverageFilter(adcFilt_type *f, uint16_t v){
	f->MA_accumulator -= f->MA_filterMas[f->MA_filterIndex];
	f->MA_filterMas[f->MA_filterIndex] = v;
	f->MA_accumulator += f->MA_filterMas[f->MA_filterIndex];

	f->MA_filterIndex++;
	if(f->MA_filterIndex >= f->MA_filter_WITH){
		f->MA_filterIndex = 0;
	}

	return f->MA_accumulator / f->MA_filter_WITH;
}

/******************************** END OF FILE ********************************/

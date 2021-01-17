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
#include <ina226.h>
#include "adcTSK.h"
#include "systemTSK.h"
#include <prmSystem.h>

/*!****************************************************************************
 * Local function declaration
 */
static inline uint16_t movingAverageFilter(adcFilt_type *f, uint16_t v);
static inline void adcTaskStctInit(void);

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
	_iq 				qtemp;
	uint8_t 			thinningCnt = 0;

    //AdcEndConversionSem
    vSemaphoreCreateBinary(AdcEndConversionSem);
    xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

    i2c_init(i2c1);
    adc_setCallback(adcHoock);
	ina226_init();
	adcTaskStctInit();

	if(ina226_verifyConnect() == INA226_OK){
		adcTaskStct.externalSensorOk = 1;
	}else{
		adcTaskStct.externalSensorOk = 0;
	}
	adc_startSampling();

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);
		for(uint8_t index = 0; index < ADC_NUM_CH; index++){
			// Oversampling
			uint32_t val = adcValue.adcreg[index] * adcTaskStct.adcFilt[index].oversampling;
			// Apply filter
			adcTaskStct.filtered[index] = movingAverageFilter(&adcTaskStct.adcFilt[index], val);
		}

		/*
		 */
		if(adcTaskStct.externalSensorOk != 0){
			if(thinningCnt == 0){
				ina226_readShuntVoltage(&adcTaskStct.adcIna226);
				ina226_trig();
			}
			thinningCnt++;
			if(thinningCnt >= 8){
				thinningCnt = 0;
			}
		}

		/*
		 * Calculate current on external ADC
		 */
		if(adcTaskStct.adcIna226 <= prm_nreadVal(Niext1_adc).t_u16Frmt){
			adcTaskStct.currentIna226 = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Niext0_adc).t_s16Frmt, prm_nreadVal(Niext1_adc).t_s16Frmt,
													IntToIQ(prm_nreadVal(Niext0_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Niext1_i).t_u32Frmt, 1000000),
													adcTaskStct.adcIna226);
		}
		else{
			adcTaskStct.currentIna226 = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Niext1_adc).t_u16Frmt, prm_nreadVal(Niext2_adc).t_u16Frmt,
													IntToIQ(prm_nreadVal(Niext1_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Niext2_i).t_u32Frmt, 1000000),
													adcTaskStct.adcIna226);
		}
		if(adcTaskStct.currentIna226 < 0){
			adcTaskStct.currentIna226 = 0;
		}

		/*
		 * Calculate voltage
		 */
		if(adcTaskStct.filtered[CH_UADC] <= prm_nreadVal(Nv1_adc).t_u16Frmt){
			adcTaskStct.voltage = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Nv0_adc).t_u16Frmt, prm_nreadVal(Nv1_adc).t_u16Frmt,
													IntToIQ(prm_nreadVal(Nv0_u).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Nv1_u).t_u32Frmt, 1000000),
													adcTaskStct.filtered[CH_UADC]);
		}
		else if(adcTaskStct.filtered[CH_UADC] <= prm_nreadVal(Nv2_adc).t_u16Frmt){
			adcTaskStct.voltage = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Nv1_adc).t_u16Frmt, prm_nreadVal(Nv2_adc).t_u16Frmt,
													IntToIQ(prm_nreadVal(Nv1_u).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Nv2_u).t_u32Frmt, 1000000),
													adcTaskStct.filtered[CH_UADC]);
		}
		else{
			adcTaskStct.voltage = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Nv2_adc).t_u16Frmt, prm_nreadVal(Nv3_adc).t_u16Frmt,
													IntToIQ(prm_nreadVal(Nv2_u).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Nv3_u).t_u32Frmt, 1000000),
													adcTaskStct.filtered[CH_UADC]);
		}
		if(adcTaskStct.voltage < 0){
			adcTaskStct.voltage = 0;
		}

		/*
		 * Detect revers voltage
		 */
//		if(adcTaskStct.adcFilt[CH_UADC].recursiveFilterOut > REVERSE_VOLTAGE_THRESHOLD){
//			adcTaskStct.reverseVoltage = 0;
//		}else{
//			adcTaskStct.reverseVoltage = 1;
//		}

		/*
		 * Calculate current
		 */
		if(adcTaskStct.filtered[CH_IADC] <= prm_nreadVal(Ni1_adc).t_u16Frmt){
			adcTaskStct.currentInt = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Ni0_adc).t_u16Frmt, prm_nreadVal(Ni1_adc).t_u16Frmt,
													IntToIQ(prm_nreadVal(Ni0_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Ni1_i).t_u32Frmt, 1000000),
													adcTaskStct.filtered[CH_IADC]);
		}
		else if(adcTaskStct.filtered[CH_IADC] <= prm_nreadVal(Ni2_adc).t_u16Frmt){
			adcTaskStct.currentInt = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Ni1_adc).t_u16Frmt, prm_nreadVal(Ni2_adc).t_u16Frmt,
													IntToIQ(prm_nreadVal(Ni1_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Ni2_i).t_u32Frmt, 1000000),
													adcTaskStct.filtered[CH_IADC]);
		}
		else{
			adcTaskStct.currentInt = s32iq_Fy_x1x2y1y2x(prm_nreadVal(Ni2_adc).t_u16Frmt, prm_nreadVal(Ni3_adc).t_u16Frmt,
													IntToIQ(prm_nreadVal(Ni2_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Ni3_i).t_u32Frmt, 1000000),
													adcTaskStct.filtered[CH_IADC]);
		}
		if(adcTaskStct.currentInt < 0){
			adcTaskStct.currentInt = 0;
		}

		/*
		 * Select current sensor
		 */
		if((adcTaskStct.currentInt > _IQ(CURRENT_SENSOR_THRESHOLD_UP / 1000.0f))
				|| (adcTaskStct.externalSensorOk == 0)){
			adcTaskStct.currentSensor = adcCurrentSensorInternal;
			adcTaskStct.current = adcTaskStct.currentInt;
		}
		if((adcTaskStct.currentInt < _IQ(CURRENT_SENSOR_THRESHOLD_DOWN / 1000.0f))
				&& (adcTaskStct.externalSensorOk != 0)){
			adcTaskStct.currentSensor = adcCcurrentSensorExternal;
			adcTaskStct.current = adcTaskStct.currentIna226;
		}

		/*
		 * Calculate input voltage
		 */
		adcTaskStct.udc = adcTaskStct.filtered[CH_UINADC] * _IQ((AdcVref * (UDC_Rh + UDC_Rl)) / (65536 * UDC_Rl));

		/*
		 * Calculate output power
		 */
		if(prm_nreadVal(Nenable).t_boolFrmt){
			adcTaskStct.outPower = _IQ14mpy(_IQtoIQ14(adcTaskStct.voltage), _IQtoIQ14(adcTaskStct.current));
		}else{
			adcTaskStct.outPower = 0;
		}

		/*
		 * Calculate dissipation power
		 */
		adcTaskStct.radPower = _IQ14mpy(_IQtoIQ14(adcTaskStct.udc - adcTaskStct.voltage),
				_IQtoIQ14(adcTaskStct.current));

		/*
		 * Calculate resistens
		 */
		if((prm_nreadVal(Nenable).t_boolFrmt) && (adcTaskStct.current > _IQ(0.001))
				&& (adcTaskStct.voltage > _IQ(0.05))){
			qtemp = _IQ14div(_IQtoIQ14(adcTaskStct.voltage), _IQtoIQ14(adcTaskStct.current));
			if(qtemp > _IQ14(99999)){  //limit 99999 Ohm
				qtemp = _IQ14(99999);
			}
			adcTaskStct.resistens = qtemp;
		}else{
			adcTaskStct.resistens = _IQ14(99999);
		}

		/*
		 * Calculate capacity
		 */
		static uint64_t capacity;
		if(prm_nreadVal(Nenable).t_boolFrmt){
			capacity += adcTaskStct.current;
			if(capacity >= ((uint64_t) _IQ(0.001) * (1000000 / adcValue.sampleRate)
							* 60 * 60)){
				adcTaskStct.capacity += 1;
				capacity = capacity
						- ((uint64_t) _IQ(0.001)
								* (1000000 / adcValue.sampleRate) * 60 * 60);
			}
		}else{
			adcTaskStct.capacity = 0;
			capacity = 0;
		}
	}
}

/*!****************************************************************************
 * @brief	Init to default adc task memory
 */
static inline void adcTaskStctInit(void){
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

/*!****************************************************************************
 * @file    		adcTSK.c
 * @author  		Storozhenko Roman - D_EL
 * @version 		V1.0.0
 * @date   		 07-01-2015
 * @copyright 	GNU Public License
 */

/*!****************************************************************************
 * Include
 */
#include "specificMath.h"
#include "adc.h"
#include "ina226.h"
#include "pstypes.h"
#include "adcTSK.h"

/*!****************************************************************************
 * Local function declaration
 */
static inline uint16_t movingAverageFilter(adcFilt_type *f, uint16_t v);
static inline void adcTaskStctInit(void);

/*!****************************************************************************
 * MEMORY
 */
adcTaskStct_type adcTaskStct = {
	{
		{ .adcDefVal = 0, .oversampling = 1, .recursiveK = 1, .MA_filter_WITH = 64, },	//U_MEAS
		{ .adcDefVal = 0, .oversampling = 1, .recursiveK = 1, .MA_filter_WITH = 64, },  //I_MEAS
		{ .adcDefVal = 0, .oversampling = 1, .recursiveK = 1, .MA_filter_WITH = 32, },  //UDC_MEAS
	},
	.externalSensorOk = internalCurrentSensor,
};

/*!****************************************************************************
 * @brief
 * @param
 * @retval
 */
void adcTSK(void *pPrm){
	uint16_t 			*adcreg = &adcStct.adcreg[0];
	uint16_t 			*adcregEnd = adcreg + ADC_NUM_CH;
	adcTaskStct_type 	*a = &adcTaskStct;
	regSetting_type 	*s = &rg.rgSet;
	adcFilt_type 		*filter = &a->adcFilt[0];
	uint16_t 			*filtered = &a->filtered[0];
	uint32_t 			val;
	_iq 				qtemp;
	uint8_t 			thinningCnt = 0;

	adcTaskStctInit();

	if(ina226_verifyConnect() == INA226_OK){
		a->externalSensorOk = 1;
	}else{
		a->externalSensorOk = 0;
	}
	startSampling();

	while(1){
		xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);

		adcreg = &adcStct.adcreg[0];
		filter = &a->adcFilt[0];
		filtered = &a->filtered[0];
		while(adcreg < adcregEnd){
			/*
			 * Передикретизация
			 */
			val = *adcreg * filter->oversampling;

//			/*
//			 * Фильтруем рекурсивным фильтром
//			 */
//			filter->recursiveFilterOut = iq_filtr(filter->recursiveFilterCumul,
//					val, filter->recursiveK);

			/*
			 * Заполнение массивов фильтра скользящего среднего
			 */
			/*filter->MA_filterMas[filter->MA_filterIndex++] = val;
			if(filter->MA_filterIndex >= filter->MA_filter_WITH){
				filter->MA_filterIndex = 0;
			}*/

			/*
			 * Фильтруем скользящим средним
			 */
			*filtered = movingAverageFilter(filter, val);

			filter++;
			filtered++;
			adcreg++;
		}

		/*
		 */
		if(a->externalSensorOk != 0){	//Если внешний сенсор найден
			if(thinningCnt == 0){
				ina226_readShuntVoltage(&a->adcIna226);
				ina226_trig();
			}
			thinningCnt++;
			if(thinningCnt >= 8){
				thinningCnt = 0;
			}
		}

		/*
		 * Рассчет измерянного тока по внешнему АЦП
		 */
		if(a->adcIna226 <= s->pIEx[1].adc){
			a->currentIna226 = s32iq_Fy_x1x2y1y2x(s->pIEx[0].adc,
					s->pIEx[1].adc, s->pIEx[0].qi, s->pIEx[1].qi, a->adcIna226);
		}else{
			a->currentIna226 = s32iq_Fy_x1x2y1y2x(s->pIEx[1].adc,
					s->pIEx[2].adc, s->pIEx[1].qi, s->pIEx[2].qi, a->adcIna226);
		}
		if(a->currentIna226 < 0)
			a->currentIna226 = 0;

		/*
		 * Рассчет измерянного напряжения
		 */
		if(a->filtered[CH_UADC] <= s->pU[1].adc){
			a->voltage = s32iq_Fy_x1x2y1y2x(s->pU[0].adc, s->pU[1].adc,
					s->pU[0].qu, s->pU[1].qu, a->filtered[CH_UADC]);
		}else if(a->filtered[CH_UADC] <= s->pU[2].adc){
			a->voltage = s32iq_Fy_x1x2y1y2x(s->pU[1].adc, s->pU[2].adc,
					s->pU[1].qu, s->pU[2].qu, a->filtered[CH_UADC]);
		}else{
			a->voltage = s32iq_Fy_x1x2y1y2x(s->pU[2].adc, s->pU[3].adc,
					s->pU[2].qu, s->pU[3].qu, a->filtered[CH_UADC]);
		}
		if(a->voltage < 0)
			a->voltage = 0;

		/*
		 * Детектор входного обратного напряжения
		 */
		/*if(a->adcFilt[CH_UADC].recursiveFilterOut > REVERSE_VOLTAGE_THRESHOLD){
			a->reverseVoltage = 0;
		}else{
			a->reverseVoltage = 1;
		}*/

		/*
		 * Рассчет измерянного тока
		 */
		if(a->filtered[CH_IADC] <= s->pU[1].adc){
			a->currentInt = s32iq_Fy_x1x2y1y2x(s->pI[0].adc, s->pI[1].adc,
					s->pI[0].qi, s->pI[1].qi, a->filtered[CH_IADC]);
		}else if(a->filtered[CH_IADC] <= s->pU[2].adc){
			a->currentInt = s32iq_Fy_x1x2y1y2x(s->pI[1].adc, s->pI[2].adc,
					s->pI[1].qi, s->pI[2].qi, a->filtered[CH_IADC]);
		}else{
			a->currentInt = s32iq_Fy_x1x2y1y2x(s->pI[2].adc, s->pI[3].adc,
					s->pI[2].qi, s->pI[3].qi, a->filtered[CH_IADC]);
		}
		if(a->currentInt < 0)
			a->currentInt = 0;

		/*
		 * Выбор текущего сенсора тока
		 */
		if((a->currentInt > _IQ(CURRENT_SENSOR_THRESHOLD_UP / 1000.0))
				|| (a->externalSensorOk == 0)){
			a->currentSensor = internalCurrentSensor;
		}
		if((a->currentInt < _IQ(CURRENT_SENSOR_THRESHOLD_DOWN / 1000.0))
				&& (a->externalSensorOk != 0)){
			a->currentSensor = externalCurrentSensor;
		}

		/*
		 * Выбор тока в качестве основного
		 */
		if(a->currentSensor == externalCurrentSensor){
			a->current = a->currentIna226;
		}else{
			a->current = a->currentInt;
		}

		/*
		 * Рассчет входного напряжения
		 */
		a->udc = a->filtered[CH_UINADC] * _IQ((AdcVref * (UDC_Rh + UDC_Rl)) / (65536 * UDC_Rl));

		/*
		 * Проверка входного напряжения
		 */
		if(a->udc >= _IQ(MIN_VIN_VOLTAGE)){
			a->lowInputVoltage = 0;
		}else{
			a->lowInputVoltage = 1;
		}

		/*
		 * Рассчет выходной мощности
		 */
		if(rg.tf.state.bit.switchIsON != 0){
			a->outPower = _IQ14mpy(_IQtoIQ14(a->voltage), _IQtoIQ14(a->current));
		}else{
			a->outPower = 0;
		}

		/*
		 * Рассчет рассеиваемой мощности
		 */
		a->radPower = _IQ14mpy(_IQtoIQ14(a->udc - a->voltage),
				_IQtoIQ14(a->current));

		/*
		 * Рассчет сопротивления нагрузки
		 */
		if((rg.tf.state.bit.switchIsON != 0) && (a->current > _IQ(0.001))
				&& (a->voltage > _IQ(0.05))){
			qtemp = _IQ14div(_IQtoIQ14(a->voltage), _IQtoIQ14(a->current));
			if(qtemp > _IQ14(99999)){  //limit 99999 Ohm
				qtemp = _IQ14(99999);
			}
			a->resistens = qtemp;
		}else{
			a->resistens = _IQ14(99999);
		}

		/*
		 * Рассчет Ah от момента включения ключа
		 */
		static uint64_t capacity;
		if(rg.tf.state.bit.switchIsON != 0){
			capacity += a->current;
			if(capacity
					>= ((uint64_t) _IQ(0.001) * (1000000 / adcStct.sampleRate)
							* 60 * 60)){
				a->capacity += 1;
				capacity = capacity
						- ((uint64_t) _IQ(0.001)
								* (1000000 / adcStct.sampleRate) * 60 * 60);
			}
		}else{
			a->capacity = 0;
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
 * @brief 	Фильтр Скользящее Среднее
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

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

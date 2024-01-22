/*!****************************************************************************
 * @file		systemTSK.c
 * @author		Storozhenko Roman - D_EL
 * @version		V1.0
 * @date		14-09-2015
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <stdio.h>
#include <assert.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <IQmathLib.h>
#include <specificMath.h>
#include <adc.h>
#include <pwm.h>
#include <flash.h>
#include <board.h>
#include <prmSystem.h>
#include <plog.h>
#include <prmSystem.h>
#include "systemTSK.h"
#include "modbusTSK.h"
#include "adcTSK.h"
#include "ds18TSK.h"

#define AdcVref							3.3		// [V]
#define UDC_Rh							47		// [kOhm]
#define UDC_Rl							2		// [kOhm]

#define REVERSE_VOLTAGE_THRESHOLD		150		// [adc lsb with oversampling]
#define CURRENT_SENSOR_THRESHOLD_UP		100		// [X_XXX A]
#define CURRENT_SENSOR_THRESHOLD_DOWN	90		// [X_XXX A]

extern "C" int _write(int fd, const void *buf, size_t count);

/*!****************************************************************************
* Memory
*/
typedef enum {
	setNone,
	setSwitchOn,
	setSwitchOff,
	setSaveSettings
} request_type;

enum{
	BASE_DAC = 0,
};

static uint16_t base[] = {
	4095
};

extern uint8_t _suser_settings;
bool currentirq;

/*!****************************************************************************
 * @brief	main output ON
 */
static inline void switchON(void){
	gppin_reset(GP_ON_OFF);
}

/*!****************************************************************************
 * @brief	main output OFF
 */
static inline void switchOFF(void){
	gppin_set(GP_ON_OFF);
}

void vsave(Prm::Val<uint32_t>& prm, bool read, void *arg){
	(void)arg;
	if(read){
		return;
	}

	switch(reinterpret_cast<uint32_t>(prm.getarg())){
		case 0:
			Prm::v0_adc = adcTaskStct.filtered.u;
			Prm::v0_dac = Prm::vdac.val;
			break;

		case 1:
			Prm::v1_adc = adcTaskStct.filtered.u;
			Prm::v1_dac = Prm::vdac.val;
			break;

		case 2:
			Prm::v2_adc = adcTaskStct.filtered.u;
			Prm::v2_dac = Prm::vdac.val;
			break;

		case 3:
			Prm::v3_adc = adcTaskStct.filtered.u;
			Prm::v3_dac = Prm::vdac.val;
			break;
	}
}

void isave(Prm::Val<uint32_t>& prm, bool read, void *arg){
	(void)arg;
	if(read){
		return;
	}

	switch(reinterpret_cast<uint32_t>(prm.getarg())){
		case 0:
			Prm::i0_adc = adcTaskStct.filtered.i;
			Prm::i0_dac = Prm::idac.val;
			Prm::iext0_adc = adcTaskStct.filtered.iex;
			Prm::iext0_i = Prm::i0_i.val;
			break;

		case 1:
			Prm::i1_adc = adcTaskStct.filtered.i;
			Prm::i1_dac = Prm::idac.val;
			Prm::iext1_adc = adcTaskStct.filtered.iex;
			Prm::iext1_i = Prm::i1_i.val;
			break;

		case 2:
			Prm::i2_adc = adcTaskStct.filtered.i;
			Prm::i2_dac = Prm::idac.val;
			Prm::iext2_adc = adcTaskStct.filtered.iex;
			Prm::iext2_i = Prm::i2_i.val;
			break;

		case 3:
			Prm::i3_adc = adcTaskStct.filtered.i;
			Prm::i3_dac = Prm::idac.val;
			Prm::iext3_adc = adcTaskStct.filtered.iex;
			Prm::iext3_i = Prm::i3_i.val;
			break;
	}
}

void irqCallback(pinMode_type *gpio){
	(void)gpio;
	currentirq = true;
}

/*!****************************************************************************
 * @brief
 */
bool savePrm(void){
	size_t settingsize = Prm::getSerialSize(Prm::savesys);
	uint8_t settingbuf[settingsize];
	Prm::serialize(Prm::savesys, settingbuf);
	taskENTER_CRITICAL();
	flash_unlock();
	flash_erasePage(&_suser_settings);
	bool memState = flash_write(&_suser_settings, (uint16_t *)settingbuf, (settingsize + 1) / sizeof(uint16_t));
	flash_lock();
	taskEXIT_CRITICAL();
	if(!memState){
		return false;
	}
	return true;
}

/*!****************************************************************************
 * @brief
 */
void systemTSK(void *pPrm){
	(void)pPrm;

	TickType_t pxPreviousWakeTime = xTaskGetTickCount();
	TickType_t lowCurrentTime = xTaskGetTickCount();
	TickType_t timeOffset = xTaskGetTickCount();
	adcTaskStct_type& a = adcTaskStct;
	uint8_t limitCnt = 0;
	bool enableState = false;
	bool reverseVoltage = false;

	plog_setVprintf(vsprintf);
	plog_setWrite(_write);
	plog_setTimestamp([]() -> uint32_t { return xTaskGetTickCount(); });

	irqSetCallback(irqCallback);

	auto settingsize = Prm::getSerialSize(Prm::savesys);
	if(!Prm::deserialize(Prm::savesys, &_suser_settings, settingsize)){
		__NOP();
	}

	auto prevenable = Prm::enable.val;

	assert(pdTRUE == xTaskCreate(modbusTSK, "modbusTSK", MODBUS_TSK_SZ_STACK,  NULL, MODBUS_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(adcTSK, "adcTSK", ADC_TSK_SZ_STACK, NULL, ADC_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(ds18TSK, "ds18TSK", DS18B_TSK_SZ_STACK, NULL, DS18B_TSK_PRIO, NULL));
	vTaskDelay(pdMS_TO_TICKS(300));

	///========================================================
	_iq					udc = 0;
	_iq					voltage = 0;			//[V]
	_iq					current = 0;			//[A]
	_iq					currentInt = 0;			//[A]
	_iq					currentExt = 0;			//[A]
	_iq14				outPower = 0;			//[W]
	_iq14				resistens = 0;			//[Ohm]
	uint32_t 			capacity = 0;			//[mAh]
	adcCurrentSensor_type	currentSensor = adcCurrentSensorInternal;


	while(1){

		///========================================================
		/*
		 * Calculate current on external ADC
		 */
		if(a.filtered.iex <= Prm::iext1_adc){
			currentExt = s32iq_Fy_x1x2y1y2x(Prm::iext0_adc, Prm::iext1_adc,
													IntToIQ(Prm::iext0_i, 1000000), IntToIQ(Prm::iext1_i, 1000000),
													a.filtered.iex);
		}
		else{
			currentExt = s32iq_Fy_x1x2y1y2x(Prm::iext1_adc, Prm::iext2_adc,
													IntToIQ(Prm::iext1_i, 1000000), IntToIQ(Prm::iext2_i, 1000000),
													a.filtered.iex);
		}
		if(currentExt < 0){
			currentExt = 0;
		}

		/*
		 * Calculate voltage
		 */
		if(a.filtered.u <= Prm::v1_adc){
			voltage = s32iq_Fy_x1x2y1y2x(Prm::v0_adc, Prm::v1_adc,
													IntToIQ(Prm::v0_u, 1000000), IntToIQ(Prm::v1_u, 1000000),
													a.filtered.u);
		}
		else if(a.filtered.u <= Prm::v2_adc){
			voltage = s32iq_Fy_x1x2y1y2x(Prm::v1_adc, Prm::v2_adc,
													IntToIQ(Prm::v1_u, 1000000), IntToIQ(Prm::v2_u, 1000000),
													a.filtered.u);
		}
		else{
			voltage = s32iq_Fy_x1x2y1y2x(Prm::v2_adc, Prm::v3_adc,
													IntToIQ(Prm::v2_u, 1000000), IntToIQ(Prm::v3_u, 1000000),
													a.filtered.u);
		}
		if(voltage < 0){
			voltage = 0;
		}

		/*
		 * Detect revers voltage
		 */
		if(a.filtered.u < REVERSE_VOLTAGE_THRESHOLD){
			reverseVoltage = true;
		}else{
			reverseVoltage = false;
		}

		/*
		 * Calculate current
		 */
		if(a.filtered.i <= Prm::i1_adc){
			currentInt = s32iq_Fy_x1x2y1y2x(Prm::i0_adc, Prm::i1_adc,
													IntToIQ(Prm::i0_i, 1000000), IntToIQ(Prm::i1_i, 1000000),
													a.filtered.i);
		}
		else if(a.filtered.i <= Prm::i2_adc){
			currentInt = s32iq_Fy_x1x2y1y2x(Prm::i1_adc, Prm::i2_adc,
													IntToIQ(Prm::i1_i, 1000000), IntToIQ(Prm::i2_i, 1000000),
													a.filtered.i);
		}
		else{
			currentInt = s32iq_Fy_x1x2y1y2x(Prm::i2_adc, Prm::i3_adc,
													IntToIQ(Prm::i2_i, 1000000), IntToIQ(Prm::i3_i, 1000000),
													a.filtered.i);
		}
		if(currentInt < 0){
			currentInt = 0;
		}

		/*
		 * Select current sensor
		 */
		if((currentInt >= _IQ(CURRENT_SENSOR_THRESHOLD_UP / 1000.0f))
				|| (a.externalSensorOk == 0)){
			currentSensor = adcCurrentSensorInternal;
		}
		if((currentInt < _IQ(CURRENT_SENSOR_THRESHOLD_DOWN / 1000.0f))
				&& (a.externalSensorOk != 0)){
			currentSensor = adcCcurrentSensorExternal;
		}

		current = currentSensor == adcCurrentSensorInternal ? currentInt : currentExt;

		/*
		 * Calculate input voltage
		 */
		udc = a.filtered.uin * _IQ((AdcVref * (UDC_Rh + UDC_Rl)) / (65536 * UDC_Rl));

		/*
		 * Calculate output power
		 */
		if(Prm::enable){
			outPower = _IQ14mpy(_IQtoIQ14(voltage), _IQtoIQ14(current));
		}else{
			outPower = 0;
		}

		/*
		 * Calculate resistance
		 */
		if(Prm::enable && (current > _IQ(0.001))
				&& (voltage > _IQ(0.05))){
			_iq qtemp = _IQ14div(_IQtoIQ14(voltage), _IQtoIQ14(current));
			if(qtemp > _IQ14(99999)){  //limit 99999 Ohm
				qtemp = _IQ14(99999);
			}
			resistens = qtemp;
		}else{
			resistens = _IQ14(99999);
		}

		/*
		 * Calculate capacity
		 */
		static uint64_t lcapacity;
		if(Prm::enable){
			lcapacity += current;
			constexpr uint32_t loopPeriod = SYSTEM_TSK_PERIOD * 1000;
			if(lcapacity >= ((uint64_t) _IQ(0.001) * (1000000 / loopPeriod) * 60 * 60)){
				capacity += 1;
				lcapacity = lcapacity - ((uint64_t) _IQ(0.001) * (1000000 / loopPeriod) * 60 * 60);
			}
		}
		if(!prevenable && Prm::enable){
			capacity = 0;
			lcapacity = 0;
		}

		prevenable = Prm::enable;

		///========================================================
		uint16_t status = Prm::status & (Prm::m_notCalibrated);

		// Temperature sensor
		if(temperature.state == temp_Ok){
			if(temperature.temperature > (TEMP_DISABLE * 10)){
				status |= Prm::m_overheated;
			}
		}
		else{
			status |= Prm::m_errorTemperatureSensor;
		}

		// Binary filter
		bool limited = false;
		const uint8_t timeConstant = 40;
		if(MODE_IS_CC()){
			if(limitCnt < timeConstant)
				limitCnt++;
			else
				limited = true;
		}
		else{
			if(limitCnt > 0)
				//limitCnt--;
				limitCnt = 0;
			else
				limited = false;
		}

		if(reverseVoltage){
			status |= Prm::m_reverseVoltage;
		}

		if(udc < _IQ(MIN_VIN_VOLTAGE)){
			status |= Prm::m_lowInputVoltage;
		}

		if(currentSensor == adcCcurrentSensorExternal){
			status |= Prm::m_externaIAdc;
		}

		if(!a.externalSensorOk){
			status |= Prm::m_errorExternalIAdc;
		}

		if(Prm::enable && limited){
			status |= Prm::m_limitation;
		}

		/**************************************
		* Set value
		*/
		Prm::vadc = a.filtered.u;
		Prm::iadc = a.filtered.i;
		Prm::iexternaladc = a.filtered.iex;
		Prm::voltage = IQtoInt(voltage, 1000000);
		Prm::current = IQtoInt(current, 1000000);
		Prm::power = IQNtoInt(outPower, 1000, 14);
		Prm::resistance = IQNtoInt(resistens, 1000, 14);
		Prm::capacity = capacity;
		Prm::input_voltage = IQtoInt(udc, 1000000);
		Prm::temperature = temperature.temperature;

		if(enableState){
			Prm::time = xTaskGetTickCount() - timeOffset;
		}

		/**************************************
		* Cooler regulator
		*/
		if(temperature.state == temp_Ok){
			_iq	qpwmTask = iq_Fy_x1x2y1y2x(_IQ(TEMP_FAN_ON), _IQ(TEMP_FAN_MAX),
										_IQ(COOLER_PWM_START), _IQ(1),
										(uint32_t)(((uint64_t)temperature.temperature << 24) / 10)
										);
			if(qpwmTask < _IQ(COOLER_PWM_START)){
				qpwmTask = _IQ(COOLER_PWM_START);
			}else if(qpwmTask > _IQ(1)){
				qpwmTask = _IQ(1);
			}

			if(temperature.temperature > (TEMP_FAN_ON * 10)){
				uint16_t pwmk = IQtoInt(qpwmTask, 1000);
				FanPwmSet(pwmk);
			}
			if(temperature.temperature < TEMP_FAN_OFF * 10){
				FanPwmSet(0);
			}
		}
		else{
			FanPwmSet(1000);
		}

		/**************************************
		* Calculate DAC values
		*/
		static uint16_t udac = 0, idac = 0;
		if(Prm::mode == Prm::dacMode){
			if(Prm::vdac <= base[BASE_DAC]){
				idac = Prm::idac;
			}else{
				idac = base[BASE_DAC];
			}
			if(Prm::vdac <= base[BASE_DAC]){
				udac = Prm::vdac;
			}else{
				udac = base[BASE_DAC];
			}
		}
		else{
			// Calc idac
			_iq qdac;
			_iq qTask = IntToIQ(Prm::current_set, 1000000);
			if(qTask == 0){
				qdac = 0;
			}
			else if(qTask <= IntToIQ(Prm::i1_i, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(Prm::i0_i, 1000000), IntToIQ(Prm::i1_i, 1000000),
						IntToIQ(Prm::i0_dac, base[BASE_DAC]), IntToIQ(Prm::i1_dac, base[BASE_DAC]),
						qTask);
			}
			else if(qTask <= IntToIQ(Prm::i2_i, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(Prm::i1_i, 1000000), IntToIQ(Prm::i2_i, 1000000),
						IntToIQ(Prm::i1_dac, base[BASE_DAC]), IntToIQ(Prm::i2_dac, base[BASE_DAC]),
						qTask);
			}
			else{
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(Prm::i2_i, 1000000), IntToIQ(Prm::i3_i, 1000000),
						IntToIQ(Prm::i2_dac, base[BASE_DAC]), IntToIQ(Prm::i3_dac, base[BASE_DAC]),
						qTask);
			}

			idac = IQtoInt(qdac, base[BASE_DAC]);
			if(idac > base[BASE_DAC]){
				idac = base[BASE_DAC];
			}

			//Calc udac
			qTask = IntToIQ(Prm::voltage_set, 1000000);
			if(qTask == 0){
				qdac = 0;
			}
			else if(qTask < IntToIQ(Prm::v1_u, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(Prm::v0_u, 1000000), IntToIQ(Prm::v1_u, 1000000),
						IntToIQ(Prm::v0_dac, base[BASE_DAC]), IntToIQ(Prm::v1_dac, base[BASE_DAC]),
						qTask);
			}
			else if(qTask < IntToIQ(Prm::v2_u, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(Prm::v1_u, 1000000), IntToIQ(Prm::v2_u, 1000000),
						IntToIQ(Prm::v1_dac, base[BASE_DAC]), IntToIQ(Prm::v2_dac, base[BASE_DAC]),
						qTask);
			}
			else{
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(Prm::v2_u, 1000000), IntToIQ(Prm::v3_u, 1000000),
						IntToIQ(Prm::v2_dac, base[BASE_DAC]), IntToIQ(Prm::v3_dac, base[BASE_DAC]),
						qTask);
			}

			udac = IQtoInt(qdac, base[BASE_DAC]);
			if(udac > base[BASE_DAC]){
				udac = base[BASE_DAC];
			}
		}

		// Setting DAC value
		setDacI(idac);
		setDacU(udac);

		if(Prm::mode == Prm::overcurrentShutdown){
			irqLimitOn();
		}
		else{
			irqLimitOff();
		}

		// Time low current
		TickType_t lowCurrentDuration = 0;
		if(enableState && Prm::current < (Prm::current_set / 10)){
			lowCurrentDuration = xTaskGetTickCount() - lowCurrentTime;
		}else{
			lowCurrentTime = xTaskGetTickCount();
		}

		Prm::mask_disablecause disablecause = Prm::v_none;
		if(status & Prm::m_errorTemperatureSensor){
			disablecause = Prm::v_errorTemperatureSensor;
		}
		else if(status & Prm::m_overheated){
			disablecause = Prm::v_overheated;
		}
		else if(status & Prm::m_lowInputVoltage){
			disablecause = Prm::v_lowInputVoltage;
		}
		else if(status & Prm::m_reverseVoltage){
			disablecause = Prm::v_reverseVoltage;
		}
		else if(currentirq || (Prm::mode == Prm::overcurrentShutdown && MODE_IS_CC())){
			disablecause = Prm::v_overCurrent;
			currentirq = false;
		}
		else if(enableState && Prm::time_set > 0 && Prm::time >= Prm::time_set){ // Disable on time
			disablecause = Prm::v_timeShutdown;
		}
		else if(Prm::mode == Prm::lowCurrentShutdown && lowCurrentDuration > pdMS_TO_TICKS(CUR_OFF_TIME)){
			disablecause = Prm::v_lowCurrentShutdown;
		}
		else if(enableState && !Prm::enable){
			disablecause = Prm::v_request;
		}

		/**************************************
		 * Request enable
		 */
		if(!enableState && Prm::enable){
			switchON();
			currentirq = false;
			enableState = true;
			timeOffset = xTaskGetTickCount();
			Prm::disablecause = Prm::v_none;
		}

		/**************************************
		 * Request disable
		 */
		if(enableState && disablecause > Prm::v_none){
			switchOFF();
			enableState = false;
			Prm::disablecause = disablecause;
			Prm::enable = false;
			disablecause = Prm::v_none;
		}

		if(status & Prm::m_lowInputVoltage && modbus_needSave(false)){
			LED_OFF();
			if(savePrm()){
				modbus_needSave(true);
			}
		}

		Prm::status = status;

		vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(SYSTEM_TSK_PERIOD));
	}
}

/*!****************************************************************************
*
*/
void OSinit(void){
	BaseType_t res = xTaskCreate(systemTSK, "systemTSK", SYSTEM_TSK_SZ_STACK, NULL, SYSTEM_TSK_PRIO, NULL);
	assert(res == pdTRUE);
	vTaskStartScheduler();
}

/******************************** END OF FILE ********************************/

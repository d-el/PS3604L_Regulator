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
#include <algorithm>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <IQmathLib.h>
#include <specificMath.h>
#include <adc.h>
#include <flash.h>
#include <board.h>
#include <prmSystem.h>
#include <plog.h>
#include <prmSystem.h>
#include "systemTSK.h"
#include "modbusTSK.h"
#include "adcTSK.h"
#include "ds18TSK.h"

#define AdcVref							(3.3)	/// [V]
#define UDC_Rh							(47)	/// [kOhm]
#define UDC_Rl							(2)		/// [kOhm]

#define SYSTEM_TSK_PERIOD				(1)		///< [ms]
#define TIME_CURRENT_OFF				(1000)	///< [ms]

#define COOLER_START					(0.3)	///< [k from maximum]
#define TEMPERATURE_FAN_OFF				(35.0)	///< [°C]
#define TEMPERATURE_FAN_ON				(43.0)	///< [°C]
#define TEMPERATURE_FAN_MAX				(60.0)	///< [°C]
#define TEMPERARURE_DISABLE				(80.0)	///< [°C]

#define MIN_VIN_VOLTAGE					(47.0)	/// [V]

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

extern uint8_t _suser_settings;
bool currentirq;

/*!****************************************************************************
 * @brief	main output ON
 */
static inline void switchON(void){
	gppin_reset(GP_ON_OFF);
}

/*!****************************************************************************
 * @brief	Set current measurement range Hi
 */
static inline void setCRangeHi(void){
	gppin_reset(GP_RNG_HI);
}

/*!****************************************************************************
 * @brief	Set current measurement range Auto
 */
static inline void setCRangeAuto(void){
	gppin_set(GP_RNG_HI);
}

/*!****************************************************************************
 * @brief	Set Voltage calibration point
 */
void vsave(Prm::Val<int32_t>& prm, bool read, void *arg){
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

/*!****************************************************************************
 * @brief	Set Current calibration point
 */
void isave(Prm::Val<int32_t>& prm, bool read, void *arg){
	(void)arg;
	if(read){
		return;
	}

	switch(reinterpret_cast<uint32_t>(prm.getarg())){
		case 0:
			Prm::i0_adc = adcTaskStct.filtered.i;
			Prm::i0_dac = Prm::idac.val;
			break;
		case 1:
			Prm::i1_adc = adcTaskStct.filtered.i;
			Prm::i1_dac = Prm::idac.val;
			break;
		case 2:
			Prm::i2_adc = adcTaskStct.filtered.i;
			Prm::i2_dac = Prm::idac.val;
			break;
		case 3:
			Prm::i3_adc = adcTaskStct.filtered.i;
			Prm::i3_dac = Prm::idac.val;
			break;
	}
}

/*!****************************************************************************
 * @brief	Set Current calibration point
 */
void micro_isave(Prm::Val<int32_t>& prm, bool read, void *arg){
	(void)arg;
	if(read){
		return;
	}

	switch(reinterpret_cast<uint32_t>(prm.getarg())){
		case 0:
			Prm::micro_iext0_adc = adcTaskStct.filtered.i;
			break;
		case 1:
			Prm::micro_iext1_adc = adcTaskStct.filtered.i;
			break;
		case 2:
			Prm::micro_iext2_adc = adcTaskStct.filtered.i;
			break;
	}
}

/*!****************************************************************************
 * @brief	CV->CC gpio interrupt handler
 */
void irqCallback(pinMode_type *gpio){
	(void)gpio;
	currentirq = true;
}

/*!****************************************************************************
 * @brief	Save parameters to flash
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
 * @brief	Main dispatcher task
 */
void systemTSK(void *pPrm){
	(void)pPrm;

	plog_setWrite(_write);
	plog_setTimestamp([]() -> uint32_t { return xTaskGetTickCount(); });

	irqSetCallback(irqCallback);

	auto settingsize = Prm::getSerialSize(Prm::savesys);
	Prm::deserialize(Prm::savesys, &_suser_settings, settingsize);
	auto prevenable = Prm::enable.val;

	BaseType_t osres = xTaskCreate(adcTSK, "adcTSK", ADC_TSK_SZ_STACK, NULL, ADC_TSK_PRIO, NULL);
	assert(osres == pdTRUE);
	osres = xTaskCreate(modbusTSK, "modbusTSK", MODBUS_TSK_SZ_STACK,  NULL, MODBUS_TSK_PRIO, NULL);
	assert(osres == pdTRUE);
	osres = xTaskCreate(ds18TSK, "ds18TSK", DS18B_TSK_SZ_STACK, NULL, DS18B_TSK_PRIO, NULL);
	assert(osres == pdTRUE);
	(void)osres;
	vTaskDelay(pdMS_TO_TICKS(300));

	///========================================================
	adcTaskStct_type&	a = adcTaskStct;
	_iq14				q20OutPower = 0;			// [W]
	uint32_t			resistance = 0;				// [Ohm]
	uint32_t			capacity = 0;				// [mAh]
	bool				irqEnable = false;
	bool				enableState = false;
	uint8_t				limitCnt = 0;
	TickType_t			timeOffset = 0;
	TickType_t			lowCurrentTime = 0;
	TickType_t			pxPreviousWakeTime = xTaskGetTickCount();
	switchON();

	while(1){
		/*
		 * Calculate current
		 */
		_iq qCurrent;
		if(/* check signal RNG_OVF */gppin_get(GP_RNG_DETECT) && Prm::crange.val == Prm::mask_crange::crange_auto){
			if(a.filtered.i <= Prm::micro_iext1_adc){
				qCurrent = s32iq_lerp(	Prm::micro_iext0_adc, IntToIQ(Prm::micro_i0_i, 1000000),
										Prm::micro_iext1_adc, IntToIQ(Prm::micro_i1_i, 1000000),
										a.filtered.i);
			}
			else{
				qCurrent = s32iq_lerp(	Prm::micro_iext1_adc, IntToIQ(Prm::micro_i1_i, 1000000),
										Prm::micro_iext2_adc, IntToIQ(Prm::micro_i2_i, 1000000),
										a.filtered.i);
			}
		}

		else{
			if(a.filtered.i <= Prm::i1_adc){
				qCurrent = s32iq_lerp(	Prm::i0_adc, IntToIQ(Prm::i0_i, 1000000),
										Prm::i1_adc, IntToIQ(Prm::i1_i, 1000000),
										a.filtered.i);
			}
			else if(a.filtered.i <= Prm::i2_adc){
				qCurrent = s32iq_lerp(	Prm::i1_adc, IntToIQ(Prm::i1_i, 1000000),
										Prm::i2_adc, IntToIQ(Prm::i2_i, 1000000),
										a.filtered.i);
			}
			else{
				qCurrent = s32iq_lerp(	Prm::i2_adc, IntToIQ(Prm::i2_i, 1000000),
										Prm::i3_adc, IntToIQ(Prm::i3_i, 1000000),
										a.filtered.i);
			}
		}

		/*
		 * Calculate voltage
		 */
		_iq qVoltage;
		if(a.filtered.u <= Prm::v1_adc){
			qVoltage = s32iq_lerp(	Prm::v0_adc, IntToIQ(Prm::v0_u, 1000000),
									Prm::v1_adc, IntToIQ(Prm::v1_u, 1000000),
									a.filtered.u);
		}
		else if(a.filtered.u <= Prm::v2_adc){
			qVoltage = s32iq_lerp(	Prm::v1_adc, IntToIQ(Prm::v1_u, 1000000),
									Prm::v2_adc, IntToIQ(Prm::v2_u, 1000000),
									a.filtered.u);
		}
		else{
			qVoltage = s32iq_lerp(	Prm::v2_adc, IntToIQ(Prm::v2_u, 1000000),
									Prm::v3_adc, IntToIQ(Prm::v3_u, 1000000),
									a.filtered.u);
		}
		_iq qWireResistens = IntToIQ(Prm::wireResistance.val, 10000);
		qVoltage  = qVoltage - _IQmpy(qWireResistens, qCurrent);

		/*
		 * Calculate input voltage
		 */
		auto calculateUin = [](uint16_t lsb) -> _iq { return lsb * _IQ((AdcVref * (UDC_Rh + UDC_Rl)) / (65536 * UDC_Rl)); };
		_iq qInVoltage = calculateUin(a.filtered.uin);

		auto caltTemparature = [](uint16_t adcVal){
			_iq qvTsh = adcVal * _IQ(AdcVref / 65536/*Full scale*/);
			_iq qTsh = _IQdiv(qvTsh - _IQ(0.4/*Static offset*/), _IQ(0.0195/*Volts per degree*/));
			return IQtoInt(qTsh, 10);
		};
		Prm::temp_shunt.val = caltTemparature(a.filtered.tsh1);
		Prm::temp_ref.val = caltTemparature(a.filtered.tsh2);
		/*
		 * Calculate output power
		 */
		if(Prm::enable){
			q20OutPower = _IQ22mpy(_IQtoIQ22(_IQabs(qVoltage)), _IQtoIQ22(_IQabs(qCurrent)));
		}else{
			q20OutPower = 0;
		}

		/*
		 * Calculate resistance
		 */
		uint32_t resistance_max = 2147483647;
		if(Prm::enable && qCurrent >= _IQ(0.00001)){
			float fv = qVoltage / float(1 << 24);
			float fi = qCurrent / float(1 << 24);
			float fr = (fv * 10000) / fi;
			fr = std::clamp(fr, (float)0, (float)resistance_max);
			if(fr <= resistance_max){
				resistance = fr;
			}else{
				resistance = -1;
			}
		}else{
			resistance = -1;
		}

		/*
		 * Calculate capacity
		 */
		static uint64_t lcapacity;
		if(Prm::enable && qCurrent > 0){
			lcapacity += qCurrent;
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
		uint16_t status = 0;

		if(Prm::calibration_time.val == 0){
			status |= Prm::m_сalibrationEmpty;
		}

		// Temperature sensor
		if(temperature.state == temp_Ok){
			if(temperature.temperature > (TEMPERARURE_DISABLE * 10)){
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
				limitCnt = 0;
			else
				limited = false;
		}

		if(qInVoltage < _IQ(9)){
			status |= Prm::m_lowInputVoltage;
		}

		if(Prm::enable && limited){
			status |= Prm::m_limitation;
		}

		/**************************************
		* Set value
		*/
		Prm::vadc = a.filtered.u;
		Prm::iadc = a.filtered.i;
		Prm::voltage = IQtoInt(qVoltage, 1000000);
		Prm::current = IQtoInt(qCurrent, 1000000);
		Prm::power = IQtoInt(q20OutPower, 1000000, 22);
		Prm::resistance = resistance;
		Prm::capacity = capacity;
		Prm::input_voltage = IQtoInt(qInVoltage, 1000000);
		Prm::temp_heatsink = temperature.temperature;
		Prm::debug_u16.val = a.filtered.vrefm;

		if(enableState){
			Prm::time.val = xTaskGetTickCount() - timeOffset;
		}

		/**************************************
		* Cooler regulator
		*/
		if(temperature.state == temp_Ok){
			_iq	qpwmTask = iq_lerp(	_IQ(TEMPERATURE_FAN_ON),_IQ(COOLER_START),
									_IQ(TEMPERATURE_FAN_MAX), _IQ(1),
									(uint32_t)(((uint64_t)temperature.temperature << 24) / 10));
			qpwmTask = _IQsat(qpwmTask, _IQ(1), _IQ(COOLER_START));
			if(temperature.temperature > (TEMPERATURE_FAN_ON * 10)){
				uint16_t pwmk = IQtoInt(qpwmTask, DAC_MAX_VALUE);
				dac_ch1(pwmk);
			}
			if(temperature.temperature < TEMPERATURE_FAN_OFF * 10){
				dac_ch1(0);
			}
		}
		else{
			dac_ch1(DAC_MAX_VALUE);
		}

		/**************************************
		* Calculate DAC values
		*/
		uint16_t udac = 0, idac = 0;
		if(Prm::mode == Prm::dacMode){
				idac = Prm::idac;
				udac = Prm::vdac;
		}
		else{
			// Calc idac
			_iq qI = IntToIQ(Prm::current_set, 1000000);
			if(qI == 0){
				idac = 0;
			}
			else if(qI <= IntToIQ(Prm::i1_i, 1000000)){
				idac = iq_lerp(	IntToIQ(Prm::i0_i, 1000000), Prm::i0_dac,
								IntToIQ(Prm::i1_i, 1000000), Prm::i1_dac,
								qI);
			}
			else if(qI <= IntToIQ(Prm::i2_i, 1000000)){
				idac = iq_lerp(	IntToIQ(Prm::i1_i, 1000000), Prm::i1_dac,
								IntToIQ(Prm::i2_i, 1000000), Prm::i2_dac,
								qI);
			}
			else{
				idac = iq_lerp(	IntToIQ(Prm::i2_i, 1000000), Prm::i2_dac,
								IntToIQ(Prm::i3_i, 1000000), Prm::i3_dac,
								qI);
			}

			// Calc udac
			_iq qU = IntToIQ(enableState ? Prm::voltage_set : 0, 1000000) + _IQmpy(qWireResistens, qCurrent);

			if(qU <= IntToIQ(Prm::v1_u, 1000000)){
				udac = iq_lerp(	IntToIQ(Prm::v0_u, 1000000), Prm::v0_dac,
								IntToIQ(Prm::v1_u, 1000000), Prm::v1_dac,
								qU);
			}
			else if(qU <= IntToIQ(Prm::v2_u, 1000000)){
				udac = iq_lerp(	IntToIQ(Prm::v1_u, 1000000), Prm::v1_dac,
								IntToIQ(Prm::v2_u, 1000000), Prm::v2_dac,
								qU);
			}
			else{
				udac = iq_lerp(	IntToIQ(Prm::v2_u, 1000000), Prm::v2_dac,
								IntToIQ(Prm::v3_u, 1000000), Prm::v3_dac,
								qU);
			}
		}

		a.dacI = idac;
		a.dacU = udac;

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
		else if(enableState && Prm::mode == Prm::overcurrentShutdown && Prm::time.val >= Prm::ocp_delay.val &&
				(currentirq || MODE_IS_CC())){
			disablecause = Prm::v_overCurrent;
			currentirq = false;
		}
		else if(enableState && Prm::time_set > 0 && Prm::time >= Prm::time_set){ // Disable on time
			disablecause = Prm::v_timeShutdown;
		}
		else if(Prm::mode == Prm::lowCurrentShutdown && lowCurrentDuration > pdMS_TO_TICKS(TIME_CURRENT_OFF)){
			disablecause = Prm::v_lowCurrentShutdown;
		}
		else if(enableState && !Prm::enable){
			disablecause = Prm::v_request;
		}

		if(Prm::mode == Prm::overcurrentShutdown && !irqEnable && enableState && Prm::time.val >= Prm::ocp_delay.val){
			irqLimitOn();
			irqEnable = true;
		}

		if(Prm::crange.val == Prm::mask_crange::crange_hi){
			setCRangeHi();
		}else{
			setCRangeAuto();
		}

		if(!gppin_get(GP_RNG_DETECT)){
			status |= Prm::mask_status::m_cRangeLoOverflow;
		}

		if(Prm::crange.val == Prm::mask_crange::crange_auto && gppin_get(GP_RNG_DETECT)){
			gppin_reset(GP_RNG_MEAS_SELECT);
		}else{
			gppin_set(GP_RNG_MEAS_SELECT);
		}

		/**************************************
		 * Request enable
		 */
		if(!enableState && Prm::enable){
			enableState = true;
			timeOffset = xTaskGetTickCount();
			Prm::disablecause = Prm::v_none;
		}

		/**************************************
		 * Request disable
		 */
		if(enableState && disablecause > Prm::v_none){
			setCRangeHi();
			irqLimitOff();
			irqEnable = false;
			enableState = false;
			Prm::disablecause = disablecause;
			Prm::enable = false;
			disablecause = Prm::v_none;
		}

		if(Prm::save_settings.val == Prm::save_do){
			if(modbus_needSave(false)){
				LED_OFF();
				if(savePrm()){
					modbus_needSave(true);
					Prm::save_settings.val = Prm::save_ok;
				}else{
					Prm::save_settings.val = Prm::save_error;
				}
			}else{
				Prm::save_settings.val = Prm::save_nothing;
			}
		}

		Prm::status = status;

		if(Prm::reboot.val == Prm::mask_reboot::reboot_do){
			vTaskDelay(pdMS_TO_TICKS(2));
			NVIC_SystemReset();
		}

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

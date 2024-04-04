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

#define AdcVref							(3.3)	/// [V]
#define UDC_Rh							(47)	/// [kOhm]
#define UDC_Rl							(2)		/// [kOhm]

#define REVERSE_VOLTAGE_THRESHOLD		(150)	/// [adc lsb with oversampling]

#define CURRENT_SENSOR_THRESHOLD_UP		(0.1)	/// [A]
#define CURRENT_SENSOR_THRESHOLD_DOWN	(0.09)	/// [A]

#define SYSTEM_TSK_PERIOD				(1)		///< [ms]
#define TIME_CURRENT_OFF				(1000)	///< [ms]

#define COOLER_PWM_START				(0.3)	///< [k PWM]
#define TEMPERATURE_FAN_OFF				(35.0)	///< [°C]
#define TEMPERATURE_FAN_ON				(43.0)	///< [°C]
#define TEMPERATURE_FAN_MAX				(60.0)	///< [°C]
#define TEMPERARURE_DISABLE				(80.0)	///< [°C]

#define MIN_VIN_VOLTAGE					(40.0)	/// [V]

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
 * @brief	main output OFF
 */
static inline void switchOFF(void){
	gppin_set(GP_ON_OFF);
}

/*!****************************************************************************
 * @brief	Set Voltage calibration point
 */
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

/*!****************************************************************************
 * @brief	Set Current calibration point
 */
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

	plog_setVprintf(vsprintf);
	plog_setWrite(_write);
	plog_setTimestamp([]() -> uint32_t { return xTaskGetTickCount(); });

	irqSetCallback(irqCallback);

	auto settingsize = Prm::getSerialSize(Prm::savesys);
	Prm::deserialize(Prm::savesys, &_suser_settings, settingsize);
	auto prevenable = Prm::enable.val;

	assert(pdTRUE == xTaskCreate(modbusTSK, "modbusTSK", MODBUS_TSK_SZ_STACK,  NULL, MODBUS_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(adcTSK, "adcTSK", ADC_TSK_SZ_STACK, NULL, ADC_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(ds18TSK, "ds18TSK", DS18B_TSK_SZ_STACK, NULL, DS18B_TSK_PRIO, NULL));
	vTaskDelay(pdMS_TO_TICKS(300));

	///========================================================
	_iq					qInVoltage = 0;				// [V]
	_iq					qVoltage = 0;				// [V]
	_iq					qCurrent = 0;				// [A]
	_iq					qCurrentInternal = 0;		// [A]
	_iq					qCurrentExternal = 0;		// [A]
	_iq					qWireResistens = 0;			// [Ohm]
	_iq14				q20OutPower = 0;			// [W]
	_iq14				q14Resistance = 0;			// [Ohm]
	uint32_t 			capacity = 0;				// [mAh]
	adcCurrentSensor_type	currentSensor = adcCurrentSensorInternal;
	bool				irqEnable = false;
	bool 				enableState = false;
	bool 				reverseVoltage = false;
	uint8_t 			limitCnt = 0;
	TickType_t			timeOffset = 0;
	TickType_t			lowCurrentTime = 0;
	TickType_t			pxPreviousWakeTime = xTaskGetTickCount();
	adcTaskStct_type& a = adcTaskStct;

	while(1){
		/*
		 * Detect reverse voltage
		 */
//		if(a.filtered.u < REVERSE_VOLTAGE_THRESHOLD){
//			reverseVoltage = true;
//		}else{
//			reverseVoltage = false;
//		}

		/*
		 * Calculate voltage
		 */
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
		qWireResistens = IntToIQ(Prm::wireResistance.val, 10000);
		qVoltage  = qVoltage - _IQmpy(qWireResistens, qCurrent);
		qVoltage = _IQsat(qVoltage, MAX_IQ_POS, 0);

		/*
		 * Calculate current
		 */
		if(a.filtered.i <= Prm::i1_adc){
			qCurrentInternal = s32iq_lerp(	Prm::i0_adc, IntToIQ(Prm::i0_i, 1000000),
											Prm::i1_adc, IntToIQ(Prm::i1_i, 1000000),
											a.filtered.i);
		}
		else if(a.filtered.i <= Prm::i2_adc){
			qCurrentInternal = s32iq_lerp(	Prm::i1_adc, IntToIQ(Prm::i1_i, 1000000),
											Prm::i2_adc, IntToIQ(Prm::i2_i, 1000000),
											a.filtered.i);
		}
		else{
			qCurrentInternal = s32iq_lerp(	Prm::i2_adc, IntToIQ(Prm::i2_i, 1000000),
											Prm::i3_adc, IntToIQ(Prm::i3_i, 1000000),
											a.filtered.i);
		}
		qCurrentInternal = _IQsat(qCurrentInternal, MAX_IQ_POS, 0);

		/*
		 * Calculate current on external ADC
		 */
		if(a.filtered.iex <= Prm::iext1_adc){
			qCurrentExternal = s32iq_lerp(	Prm::iext0_adc, IntToIQ(Prm::iext0_i, 1000000),
											Prm::iext1_adc, IntToIQ(Prm::iext1_i, 1000000),
											a.filtered.iex);
		}
		else{
			qCurrentExternal = s32iq_lerp(	Prm::iext1_adc, IntToIQ(Prm::iext1_i, 1000000),
											Prm::iext2_adc, IntToIQ(Prm::iext2_i, 1000000),
											a.filtered.iex);
		}
		qCurrentExternal = _IQsat(qCurrentExternal, MAX_IQ_POS, 0);

		/*
		 * Select current sensor
		 */
		if((qCurrentInternal >= _IQ(CURRENT_SENSOR_THRESHOLD_UP))
				|| (a.externalSensorOk == 0)){
			currentSensor = adcCurrentSensorInternal;
		}
		if((qCurrentInternal < _IQ(CURRENT_SENSOR_THRESHOLD_DOWN))
				&& (a.externalSensorOk != 0)){
			currentSensor = adcCcurrentSensorExternal;
		}

		qCurrent = currentSensor == adcCurrentSensorInternal ? qCurrentInternal : qCurrentExternal;

		/*
		 * Calculate input voltage
		 */
		qInVoltage = a.filtered.uin * _IQ((AdcVref * (UDC_Rh + UDC_Rl)) / (65536 * UDC_Rl));

		/*
		 * Calculate output power
		 */
		if(Prm::enable){
			q20OutPower = _IQ20mpy(_IQtoIQ20(qVoltage), _IQtoIQ20(qCurrent));
		}else{
			q20OutPower = 0;
		}

		/*
		 * Calculate resistance
		 */
		if(Prm::enable && (qCurrent > _IQ(0.01))
				/*&& (voltage > _IQ(0.05))*/){
			q14Resistance = _IQ14div(_IQtoIQ14(qVoltage), _IQtoIQ14(qCurrent));
			if(q14Resistance > _IQ14(99999)){  //limit 99999 Ohm
				q14Resistance = _IQ14(99999);
			}
		}else{
			q14Resistance = _IQ14(99999);
		}

		/*
		 * Calculate capacity
		 */
		static uint64_t lcapacity;
		if(Prm::enable){
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
			status |= Prm::m_notCalibrated;
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

		if(reverseVoltage){
			status |= Prm::m_reverseVoltage;
		}

		if(qInVoltage < _IQ(MIN_VIN_VOLTAGE)){
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
		Prm::voltage = IQtoInt(qVoltage, 1000000);
		Prm::current = IQtoInt(qCurrent, 1000000);
		Prm::power = IQtoInt(q20OutPower, 1000, 20);
		Prm::resistance = IQtoInt(q14Resistance, 10000, 14);
		Prm::capacity = capacity;
		Prm::input_voltage = IQtoInt(qInVoltage, 1000000);
		Prm::temperature = temperature.temperature;

		if(enableState){
			Prm::time.val = xTaskGetTickCount() - timeOffset;
		}

		/**************************************
		* Cooler regulator
		*/
		if(temperature.state == temp_Ok){
			_iq	qpwmTask = iq_lerp(	_IQ(TEMPERATURE_FAN_ON),_IQ(COOLER_PWM_START),
									_IQ(TEMPERATURE_FAN_MAX), _IQ(1),
									(uint32_t)(((uint64_t)temperature.temperature << 24) / 10));
			qpwmTask = _IQsat(qpwmTask, _IQ(1), _IQ(COOLER_PWM_START));
			if(temperature.temperature > (TEMPERATURE_FAN_ON * 10)){
				uint16_t pwmk = IQtoInt(qpwmTask, 1000);
				FanPwmSet(pwmk);
			}
			if(temperature.temperature < TEMPERATURE_FAN_OFF * 10){
				FanPwmSet(0);
			}
		}
		else{
			FanPwmSet(1000);
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
			_iq qU = IntToIQ(Prm::voltage_set, 1000000) + _IQmpy(qWireResistens, qCurrent);

			if(qU == 0){
				udac = 0;
			}
			else if(qU < IntToIQ(Prm::v1_u, 1000000)){
				udac = iq_lerp(	IntToIQ(Prm::v0_u, 1000000), Prm::v0_dac,
								IntToIQ(Prm::v1_u, 1000000), Prm::v1_dac,
								qU);
			}
			else if(qU < IntToIQ(Prm::v2_u, 1000000)){
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
		else if(status & Prm::m_reverseVoltage){
			disablecause = Prm::v_reverseVoltage;
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

		/**************************************
		 * Request enable
		 */
		if(!enableState && Prm::enable){
			switchON();
			enableState = true;
			timeOffset = xTaskGetTickCount();
			Prm::disablecause = Prm::v_none;
		}

		/**************************************
		 * Request disable
		 */
		if(enableState && disablecause > Prm::v_none){
			switchOFF();
			irqLimitOff();
			irqEnable = false;
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

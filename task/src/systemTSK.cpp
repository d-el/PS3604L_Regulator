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
#include <printp.h>
#include <prmSystem.h>
#include "systemTSK.h"
#include "modbusTSK.h"
#include "adcTSK.h"
#include "ds18TSK.h"

/*-------NAME--------------------size [4 byte Word] */
#define SYSTEM_TSK_SZ_STACK     512
#define ADC_TSK_SZ_STACK        512
#define UART_TSK_SZ_STACK       512
#define DS18B_TSK_SZ_STACK      512
/*-------NAME--------------------size [4 byte Word] */
#define SYSTEM_TSK_PRIO         3
#define ADC_TSK_PRIO            4
#define UART_TSK_PRIO           2
#define DS18B_TSK_PRIO          1


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
bool flagsave;

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
			Prm::v0_adc = adcTaskStct.filtered[CH_UADC];
			Prm::v0_dac = Prm::vdac.val;
			break;

		case 1:
			Prm::v1_adc = adcTaskStct.filtered[CH_UADC];
			Prm::v1_dac = Prm::vdac.val;
			break;

		case 2:
			Prm::v2_adc = adcTaskStct.filtered[CH_UADC];
			Prm::v2_dac = Prm::vdac.val;
			break;

		case 3:
			Prm::v3_adc = adcTaskStct.filtered[CH_UADC];
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
			Prm::i0_adc = adcTaskStct.filtered[CH_IADC];
			Prm::i0_dac = Prm::idac.val;
			Prm::iext0_adc = adcTaskStct.adcIna229;
			Prm::iext0_i = Prm::i0_i.val;
			break;

		case 1:
			Prm::i1_adc = adcTaskStct.filtered[CH_IADC];
			Prm::i1_dac = Prm::idac.val;
			Prm::iext1_adc = adcTaskStct.adcIna229;
			Prm::iext1_i = Prm::i1_i.val;
			break;

		case 2:
			Prm::i2_adc = adcTaskStct.filtered[CH_IADC];
			Prm::i2_dac = Prm::idac.val;
			Prm::iext2_adc = adcTaskStct.adcIna229;
			Prm::iext2_i = Prm::i2_i.val;
			break;

		case 3:
			Prm::i3_adc = adcTaskStct.filtered[CH_IADC];
			Prm::i3_dac = Prm::idac.val;
			Prm::iext3_adc = adcTaskStct.adcIna229;
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
	flash_unlock();
	flash_erasePage(&_suser_settings);
	flashState_type memState = flash_write(&_suser_settings, (uint16_t *)settingbuf, (settingsize + 1) /sizeof(uint16_t));
	flash_lock();
	if(memState != flash_ok){
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
	TickType_t curOffTime = xTaskGetTickCount();
	TickType_t timeOffset = xTaskGetTickCount();
	adcTaskStct_type *a = &adcTaskStct;
	uint8_t limitCnt = 0;

	print_init(stdOut_semihost);

	irqSetCallback(irqCallback);

	auto settingsize = Prm::getSerialSize(Prm::savesys);
	if(!Prm::deserialize(Prm::savesys, &_suser_settings, settingsize)){
		__NOP();
	}

	assert(pdTRUE == xTaskCreate(modbusTSK, "uartTSK", UART_TSK_SZ_STACK,  NULL, UART_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(adcTSK, "adcTSK", ADC_TSK_SZ_STACK, NULL, ADC_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(ds18TSK, "ds18TSK", DS18B_TSK_SZ_STACK, NULL, DS18B_TSK_PRIO, NULL));
	vTaskDelay(pdMS_TO_TICKS(300));

	while(1){
		uint16_t state = Prm::state & (Prm::m_notCalibrated);

		// Temperature sensor
		if(temperature.state == temp_Ok){
			if(temperature.temperature > (TEMP_OFF * 10)){
				state |= Prm::m_overheated;
			}
		}
		else{
			state |= Prm::m_errorTemperatureSensor;
		}

		// Binary filter
		bool limited = false;
		const uint8_t numbersample = 10;
		if(MODE_IS_CC()){
			if(limitCnt < numbersample)
				limitCnt++;
			else
				limited = true;
		}
		else{
			if(limitCnt > 0)
				limitCnt--;
			else
				limited = false;
		}


		if(currentirq){
			state |= Prm::m_overCurrent;
		}

		if(a->reverseVoltage){
			state |= Prm::m_reverseVoltage;
		}

		if(a->udc < _IQ(MIN_VIN_VOLTAGE)){
			state |= Prm::m_lowInputVoltage;
		}

		if(a->currentSensor == adcCcurrentSensorExternal){
			state |= Prm::m_externaIDac;
		}

		if(Prm::enable && limited){
			state |= Prm::m_limitation;
		}

		/**************************************
		* Set value
		*/
		Prm::vadc = a->filtered[CH_UADC];
		Prm::iadc = a->filtered[CH_IADC];
		Prm::iexternaladc = a->adcIna229;
		Prm::voltage = IQtoInt(a->voltage, 1000000);
		Prm::current = IQtoInt(a->current, 1000000);
		Prm::power = IQNtoInt(a->outPower, 1000, 14);
		Prm::resistance = IQNtoInt(a->resistens, 1000, 14);
		Prm::capacity = a->capacity;
		Prm::input_voltage = IQtoInt(a->udc, 1000000);
		Prm::temperature = temperature.temperature;

		if(Prm::enable){
			Prm::time = (xTaskGetTickCount() - timeOffset) / configTICK_RATE_HZ;
		}else{
			Prm::time = 0;
		}

		/**************************************
		* Cooler regulator
		*/
		if(temperature.state == temp_Ok){
			_iq	qpwmTask = iq_Fy_x1x2y1y2x(_IQ(MIN_TEMP), _IQ(MAX_TEMP),
									   _IQ(COOLER_PWM_START), _IQ(1),
									   (uint32_t)(((uint64_t)temperature.temperature << 24) / 10)
									   );
			if(qpwmTask < _IQ(COOLER_PWM_START)){
				qpwmTask = _IQ(COOLER_PWM_START);
			}else if(qpwmTask > _IQ(1)){
				qpwmTask = _IQ(1);
			}

			if(temperature.temperature > (MIN_TEMP * 10)){
				uint16_t pwmk = IQtoInt(qpwmTask, 1000);
				FanPwmSet(pwmk);
			}
			if(temperature.temperature < ((MIN_TEMP - H_TEMP) * 10)){
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

		request_type l_switchRequest = setNone;

		// Disable on time
		if(Prm::mode == Prm::timeShutdown){
			if((Prm::time >= Prm::time_set) && Prm::enable){
				l_switchRequest = setSwitchOff;
			}
		}

		/**************************************
		 *
		 */
		if(Prm::mode == Prm::overcurrentShutdown){
			if(MODE_IS_CC()){
				l_switchRequest = setSwitchOff;
			}
		}

		// Low current disable
		if(Prm::mode == Prm::lowCurrentShutdown){
			if(((Prm::current <= (Prm::current / 10))
					&&(Prm::enable))||(Prm::current == 0))
			{
				if((xTaskGetTickCount() - curOffTime) > pdMS_TO_TICKS(CUR_OFF_TIME)){
					l_switchRequest = setSwitchOff;
				}
			}else{
				curOffTime = xTaskGetTickCount();
			}
		}

		/**************************************
		 * On fails disable output
		 */
		if(state & (Prm::m_overheated | Prm::m_errorTemperatureSensor | Prm::m_lowInputVoltage | Prm::m_reverseVoltage)){
			l_switchRequest = setSwitchOff;
		}

		/**************************************
		 * Request enable
		 */
		static bool enable = false;
		if(!enable && Prm::enable){
			enable = true;
			l_switchRequest = setSwitchOn;
		}
		else if(enable && !Prm::enable){
			enable = false;
			l_switchRequest = setSwitchOff;
		}

		/**************************************
		 * Enabling / Disabling
		 */
		if(l_switchRequest == setSwitchOn){
			currentirq = false;
			setDacU(0);
			switchON();
			timeOffset = xTaskGetTickCount();
			l_switchRequest = setNone;
		}
		else if(l_switchRequest == setSwitchOff){
			setDacU(0);
			switchOFF();
			l_switchRequest = setNone;
			Prm::enable = false;
		}

		if((state & Prm::m_lowInputVoltage || flagsave) && modbus_needSave(false)){
			if(savePrm()){
				modbus_needSave(true);
			}
		}

		Prm::state = state;

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

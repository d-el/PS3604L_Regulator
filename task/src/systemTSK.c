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
#define ADC_TSK_SZ_STACK        512
#define UART_TSK_SZ_STACK       512
#define DS18B_TSK_SZ_STACK      512
/*-------NAME--------------------size [4 byte Word] */

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

uint16_t base[] = {
	4095
};

bool currentirq;
extern uint8_t _suser_settings;

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

void vsave(const struct prmHandle* h, void *arg){
	if(arg == NULL){
		return;
	}
	if(h->parameter > 3){
		return;
	}
	prm_writeVal(prm_getHandler(Nv0_adc + h->parameter), (prmval_type){ .t_u16Frmt = adcTaskStct.filtered[CH_UADC] }, NULL);
	prmval_type dac = prm_nreadVal(Nvdac);
	prm_writeVal(prm_getHandler(Nv0_dac + h->parameter), (prmval_type){ .t_u16Frmt = dac.t_u16Frmt }, NULL);
}

void isave(const struct prmHandle* h, void *arg){
	if(arg == NULL){
		return;
	}
	if(h->parameter > 3){
		return;
	}
	prm_writeVal(prm_getHandler(Ni0_adc + h->parameter), (prmval_type){ .t_u16Frmt = adcTaskStct.filtered[CH_IADC] }, NULL);
	prmval_type dac = prm_nreadVal(Nidac);
	prm_writeVal(prm_getHandler(Ni0_dac + h->parameter), (prmval_type){ .t_u16Frmt = dac.t_u16Frmt }, NULL);

	prm_writeVal(prm_getHandler(Niext0_i + h->parameter), *prm_getHandler(Ni0_i + h->parameter)->prm, NULL);
	prm_writeVal(prm_getHandler(Niext0_adc + h->parameter), (prmval_type){ .t_u16Frmt = adcTaskStct.adcIna226 }, NULL);
}

void irqCallback(pinMode_type *gpio){
	(void)gpio;
	currentirq = true;
}

/*!****************************************************************************
 * @brief
 */
bool savePrm(void){
	size_t settingsize = prm_size(prmSaveSys);
	uint8_t settingbuf[settingsize];
	size_t serialisedsize;
	prm_serialize(settingbuf, &serialisedsize, prmSaveSys);
	flash_unlock();
	flash_erasePage(&_suser_settings);
	flashState_type memState = flash_write(&_suser_settings, (uint16_t *)settingbuf, serialisedsize/sizeof(uint16_t));
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

	prm_loadDefault(prmNotSave);

	if(prm_deserialize(&_suser_settings, prmSaveSys) != prm_ok){
		prm_writeVal(prm_getHandler(Nstate), (prmval_type){ .t_u16Frmt = m_notCalibrated }, NULL);
		prm_loadDefault(prmSaveSys);
	}

	assert(pdTRUE == xTaskCreate(modbusTSK, "uartTSK", UART_TSK_SZ_STACK,  NULL, UART_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(adcTSK, "adcTSK", ADC_TSK_SZ_STACK, NULL, ADC_TSK_PRIO, NULL));
	assert(pdTRUE == xTaskCreate(ds18TSK, "ds18TSK", DS18B_TSK_SZ_STACK, NULL, DS18B_TSK_PRIO, NULL));

	vTaskDelay(pdMS_TO_TICKS(200));

	while(1){
		uint16_t state = prm_nreadVal(Nstate).t_u16Frmt & (m_notCalibrated);

		// Temperature sensor
		if(temperature.state == temp_Ok){
			if(temperature.temperature > (TEMP_OFF * 10)){
				state |= m_overheated;
			}
		}
		else{
			state |= m_errorTemperatureSensor;
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
			state |= m_overCurrent;
		}

		if(a->reverseVoltage){
			state |= m_reverseVoltage;
		}

		if(a->udc < _IQ(MIN_VIN_VOLTAGE)){
			state |= m_lowInputVoltage;
		}

		if(a->currentSensor == adcCcurrentSensorExternal){
			state |= m_externaIDac;
		}

		if(prm_nreadVal(Nenable).t_boolFrmt && limited){
			state |= m_limitation;
		}

		/**************************************
		* Set value
		*/
		prm_writeVal(prm_getHandler(Nvadc), (prmval_type){ .t_u16Frmt = a->filtered[CH_UADC] }, NULL);
		prm_writeVal(prm_getHandler(Niadc), (prmval_type){ .t_u16Frmt = a->filtered[CH_IADC] }, NULL);
		prm_writeVal(prm_getHandler(Niexternaladc), (prmval_type){ .t_u16Frmt = a->adcIna226 }, NULL);
		prm_writeVal(prm_getHandler(Nvoltage), (prmval_type){ .t_u32Frmt = IQtoInt(a->voltage, 1000000) }, NULL);
		prm_writeVal(prm_getHandler(Ncurrent), (prmval_type){ .t_u32Frmt = IQtoInt(a->current, 1000000) }, NULL);
		prm_writeVal(prm_getHandler(Npower), (prmval_type){ .t_u32Frmt = IQNtoInt(a->outPower, 1000, 14) }, NULL);
		prm_writeVal(prm_getHandler(Nresistance), (prmval_type){ .t_u32Frmt = IQNtoInt(a->resistens, 1000, 14) }, NULL);
		prm_writeVal(prm_getHandler(Ncapacity), (prmval_type){ .t_u32Frmt = a->capacity }, NULL);
		prm_writeVal(prm_getHandler(Ninput_voltage), (prmval_type){ .t_u32Frmt = IQtoInt(a->udc, 1000000) }, NULL);
		prm_writeVal(prm_getHandler(Ntemperature), (prmval_type){ .t_u16Frmt = temperature.temperature }, NULL);

		prmval_type prmval = {};
		if(prm_readVal(prm_getHandler(Nenable)).t_boolFrmt == true){
			prmval = (prmval_type){ .t_u32Frmt = (xTaskGetTickCount() - timeOffset) / configTICK_RATE_HZ };
		}
		prm_writeVal(prm_getHandler(Ntime), prmval, NULL);

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
		if(prm_nreadVal(Nmode).t_u16Frmt == dacMode){
			if(prm_nreadVal(Nvdac).t_u16Frmt <= base[BASE_DAC]){
				idac = prm_nreadVal(Nidac).t_u16Frmt;
			}else{
				idac = base[BASE_DAC];
			}
			if(prm_nreadVal(Nvdac).t_u16Frmt <= base[BASE_DAC]){
				udac = prm_nreadVal(Nvdac).t_u16Frmt;
			}else{
				udac = base[BASE_DAC];
			}
		}
		else{
			// Calc idac
			_iq qdac;
			_iq qTask = IntToIQ(prm_nreadVal(Ncurrent_set).t_u32Frmt, 1000000);
			if(qTask == 0){
				qdac = 0;
			}
			else if(qTask <= IntToIQ(prm_nreadVal(Ni1_i).t_u32Frmt, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(prm_nreadVal(Ni0_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Ni1_i).t_u32Frmt, 1000000),
						IntToIQ(prm_nreadVal(Ni0_dac).t_u16Frmt, base[BASE_DAC]), IntToIQ(prm_nreadVal(Ni1_dac).t_u16Frmt, base[BASE_DAC]),
						qTask);
			}
			else if(qTask <= IntToIQ(prm_nreadVal(Ni2_i).t_u32Frmt, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(prm_nreadVal(Ni1_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Ni2_i).t_u32Frmt, 1000000),
						IntToIQ(prm_nreadVal(Ni1_dac).t_u16Frmt, base[BASE_DAC]), IntToIQ(prm_nreadVal(Ni2_dac).t_u16Frmt, base[BASE_DAC]),
						qTask);
			}
			else{
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(prm_nreadVal(Ni2_i).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Ni3_i).t_u32Frmt, 1000000),
						IntToIQ(prm_nreadVal(Ni2_dac).t_u16Frmt, base[BASE_DAC]), IntToIQ(prm_nreadVal(Ni3_dac).t_u16Frmt, base[BASE_DAC]),
						qTask);
			}

			idac = IQtoInt(qdac, base[BASE_DAC]);
			if(idac > base[BASE_DAC]){
				idac = base[BASE_DAC];
			}

			//Calc udac
			qTask = IntToIQ(prm_nreadVal(Nvoltage_set).t_u32Frmt, 1000000);
			if(qTask == 0){
				qdac = 0;
			}
			else if(qTask < IntToIQ(prm_nreadVal(Nv1_u).t_u32Frmt, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(prm_nreadVal(Nv0_u).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Nv1_u).t_u32Frmt, 1000000),
						IntToIQ(prm_nreadVal(Nv0_dac).t_u16Frmt, base[BASE_DAC]), IntToIQ(prm_nreadVal(Nv1_dac).t_u16Frmt, base[BASE_DAC]),
						qTask);
			}
			else if(qTask < IntToIQ(prm_nreadVal(Nv2_u).t_u32Frmt, 1000000)){
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(prm_nreadVal(Nv1_u).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Nv2_u).t_u32Frmt, 1000000),
						IntToIQ(prm_nreadVal(Nv1_dac).t_u16Frmt, base[BASE_DAC]), IntToIQ(prm_nreadVal(Nv2_dac).t_u16Frmt, base[BASE_DAC]),
						qTask);
			}
			else{
				qdac = iq_Fy_x1x2y1y2x(IntToIQ(prm_nreadVal(Nv2_u).t_u32Frmt, 1000000), IntToIQ(prm_nreadVal(Nv3_u).t_u32Frmt, 1000000),
						IntToIQ(prm_nreadVal(Nv2_dac).t_u16Frmt, base[BASE_DAC]), IntToIQ(prm_nreadVal(Nv3_dac).t_u16Frmt, base[BASE_DAC]),
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

		if(prm_nreadVal(Nmode).t_u16Frmt == overcurrentShutdown){
			irqLimitOn();
		}
		else{
			irqLimitOff();
		}

		request_type l_switchRequest = setNone;

		// Disable on time
		if(prm_nreadVal(Nmode).t_u16Frmt == timeShutdown){
			if((prm_nreadVal(Ntime).t_u16Frmt >= prm_nreadVal(Ntime_set).t_u16Frmt) && prm_nreadVal(Nenable).t_u16Frmt){
				l_switchRequest = setSwitchOff;
			}
		}

		/**************************************
		 *
		 */
		if(prm_nreadVal(Nmode).t_u16Frmt == overcurrentShutdown){
			if(MODE_IS_CC()){
				l_switchRequest = setSwitchOff;
			}
		}

		// Low current disable
		if(prm_nreadVal(Nmode).t_u16Frmt == lowCurrentShutdown){
			if(((prm_nreadVal(Ncurrent).t_u32Frmt <= (prm_nreadVal(Ncurrent).t_u32Frmt / 10))
					&&(prm_nreadVal(Nenable).t_boolFrmt))||(prm_nreadVal(Ncurrent).t_u32Frmt == 0))
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
		if(state & (m_overheated | m_errorTemperatureSensor | m_lowInputVoltage | m_reverseVoltage)){
			l_switchRequest = setSwitchOff;
		}

		/**************************************
		 * Request enable
		 */
		static bool enable = false;
		if(enable == false && prm_nreadVal(Nenable).t_boolFrmt == true){
			enable = true;
			l_switchRequest = setSwitchOn;
		}
		else if(enable == true && prm_nreadVal(Nenable).t_boolFrmt == false){
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
			prm_writeVal(prm_getHandler(Nenable), (prmval_type){ .t_boolFrmt = false }, NULL);
		}

		if(state & m_lowInputVoltage && modbus_needSave(false)){
			if(savePrm()){
				modbus_needSave(true);
			}
		}

		prm_writeVal(prm_getHandler(Nstate), (prmval_type){ .t_u16Frmt = state }, NULL);

		vTaskDelayUntil(&pxPreviousWakeTime, pdMS_TO_TICKS(5));
	}
}

/******************************** END OF FILE ********************************/

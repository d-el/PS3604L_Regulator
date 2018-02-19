/*!****************************************************************************
* @file    		systemTSK.c
* @author  		Storozhenko Roman - D_EL
* @version		 V1.0
* @date    		14-09-2015
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "IQmathLib.h"
#include "specificMath.h"
#include "adc.h"
#include "pwm.h"
#include "flash.h"
#include "board.h"
#include "OSinit.h"
#include "pstypes.h"
#include "prmSystem.h"
#include "systemTSK.h"
#include "uartTSK.h"
#include "adcTSK.h"
#include "ds18TSK.h"

/*!****************************************************************************
* Memory
*/
uint16_t pwmk;
regulator_type      rg;
uint16_t            base[] = {
    4095
};

/*!****************************************************************************
* @brief    main output ON
*/
static inline void switchON(void){
    gppin_reset(GP_ON_OFF);
    rg.tf.state.bit.switchIsON = 1;
}

/*!****************************************************************************
* @brief    main output OFF
*/
static inline void switchOFF(void){
    gppin_set(GP_ON_OFF);
    rg.tf.state.bit.switchIsON = 0;
}

/*!****************************************************************************
*
*/
void systemTSK(void *pPrm){
	BaseType_t  		osRes;
	TickType_t			curOffTime = xTaskGetTickCount();	//Время тока < 10%
	TickType_t          timeOffset = xTaskGetTickCount();
    regulator_type      *reg = &rg;
    regSetting_type     *s = &reg->sett; 		//Указатель на структуру с калибровками
    adcTaskStct_type    *a = &adcTaskStct;    	//Указатель на сруктуру с данными АЦП
    request_type        l_switchRequest;    	//Локальный запрос на вкл / выкл выход
    _iq                 qpwmTask;           	//Заполнение ШИМ для венитлятора
    _iq                 qTask, qdac;

    uint16_t            idac, udac;

    l_switchRequest     = setSwitchOff;

    while(1){
    	osRes = xSemaphoreTake(rxRequest, pdMS_TO_TICKS(MAX_WAIT_RxRequest));

        /**************************************
        * Вызов периодических функций
        */
    	static uint8_t ledCount = 0;
		if(ledCount++ == 100){
			LED_ON();
			ledCount = 0;
		}
		if(ledCount == 10){
			LED_OFF();
		}
		if((ledCount == 20)&&(osRes == pdTRUE)){
			LED_ON();
		}
		if(ledCount == 30){
			LED_OFF();
		}

        /**************************************
        * Анализ датчика температуры линейного регулятора
        */
        if(temperature.state == temp_Ok){
            if(temperature.temperature > (TEMP_OFF * 10)){
                reg->tf.state.bit.ovfLinearRegTemper = 1;        //Превышение температуры линейного стабилизатора
            }else{
                reg->tf.state.bit.ovfLinearRegTemper = 0;
            }
            reg->tf.state.bit.errorLinearRegTemperSens = 0;
        }
        else{
            reg->tf.state.bit.errorLinearRegTemperSens = 1;
        }

        /**************************************
        * Проверяем на режим ограничения тока
        */
        if(reg->tf.state.bit.switchIsON != 0){
           reg->tf.state.bit.modeIlim = MODE_IS_CC();
        }else{
            reg->tf.state.bit.modeIlim = 0;
        }

        /**************************************
        * Перекладываем значения с модуля измерителя
        */
		reg->tf.meas.adcu 			= a->filtered[CH_UADC];
		reg->tf.meas.adci 			= a->filtered[CH_IADC];
		reg->tf.meas.u 				= IQtoInt(a->voltage, 1000000);
		reg->tf.meas.i 				= IQtoInt(a->current, 1000000);
        reg->tf.meas.resistance      = IQNtoInt(a->resistens, 1, 14);
        reg->tf.meas.power          = IQNtoInt(a->outPower, 1000, 14);
        reg->tf.meas.uin            = IQtoInt(a->udc, 1000);
        reg->tf.meas.temperatureLin = temperature.temperature;
        reg->tf.meas.capacity       = a->capacity;
        reg->tf.state.bit.reverseVoltage = a->reverseVoltage;
        reg->tf.state.bit.lowInputVoltage = a->lowInputVoltage;

        if(reg->tf.state.bit.switchIsON != 0){
            reg->tf.meas.time = (xTaskGetTickCount() - timeOffset) / configTICK_RATE_HZ;
        }

        /**************************************
        * Регулятор вентилятора
        */
        qpwmTask = iq_Fy_x1x2y1y2x(_IQ(MIN_TEMP), _IQ(MAX_TEMP),
                                   _IQ(COOLER_PWM_START), _IQ(1),
                                   (uint32_t)(((uint64_t)temperature.temperature << 24) / 10)
                                   );
        if(qpwmTask < _IQ(COOLER_PWM_START)){
            qpwmTask = _IQ(COOLER_PWM_START);
        }else if(qpwmTask > _IQ(1)){
            qpwmTask = _IQ(1);
        }

        if(temperature.temperature > (MIN_TEMP * 10)){
        	pwmk = IQtoInt(qpwmTask, 1000);
        	FanPwmSet(pwmk);
        }
        if(temperature.temperature < ((MIN_TEMP - H_TEMP) * 10)){
        	FanPwmSet(0);
        }

        /**************************************
        * Рассчет значения ЦАП
        */
        switch(reg->tf.task.mode){
        	case mode_overcurrentShutdown:
        	case mode_limitation:
        	case mode_timeShutdown:
        	case mode_lowCurrentShutdown:
        		//Calc idac
        		qTask = IntToIQ(reg->tf.task.i, 1000000);
				if(qTask == 0){
					qdac = 0;
				}
				else if(qTask <= s->pI[1].qi){
					qdac = iq_Fy_x1x2y1y2x(s->pI[0].qi, s->pI[1].qi, IntToIQ(s->pI[0].dac, base[BASE_DAC]), IntToIQ(s->pI[1].dac, base[BASE_DAC]), qTask);
				}
				else if(qTask <= s->pI[2].qi){
					qdac = iq_Fy_x1x2y1y2x(s->pI[1].qi, s->pI[2].qi, IntToIQ(s->pI[1].dac, base[BASE_DAC]), IntToIQ(s->pI[2].dac, base[BASE_DAC]), qTask);
				}
				else{
					qdac = iq_Fy_x1x2y1y2x(s->pI[2].qi, s->pI[3].qi, IntToIQ(s->pI[2].dac, base[BASE_DAC]), IntToIQ(s->pI[3].dac, base[BASE_DAC]), qTask);
				}
				idac = IQtoInt(qdac, base[BASE_DAC]);
				if(idac > base[BASE_DAC]){
					idac = base[BASE_DAC];
				}

				//Calc udac
				qTask = IntToIQ(reg->tf.task.u, 1000000);
				if(qTask == 0){
					qdac = 0;
				}
				else if(qTask < s->pU[1].qu){
					qdac = iq_Fy_x1x2y1y2x(s->pU[0].qu, s->pU[1].qu, IntToIQ(s->pU[0].dac, base[BASE_DAC]), IntToIQ(s->pU[1].dac, base[BASE_DAC]), qTask);
				}
				else if(qTask < s->pU[2].qu){
					qdac = iq_Fy_x1x2y1y2x(s->pU[1].qu, s->pU[2].qu, IntToIQ(s->pU[1].dac, base[BASE_DAC]), IntToIQ(s->pU[2].dac, base[BASE_DAC]), qTask);
				}
				else{
					qdac = iq_Fy_x1x2y1y2x(s->pU[2].qu, s->pU[3].qu, IntToIQ(s->pU[2].dac, base[BASE_DAC]), IntToIQ(s->pU[3].dac, base[BASE_DAC]), qTask);
				}

				udac = IQtoInt(qdac, base[BASE_DAC]);
				if(udac > base[BASE_DAC]){
					udac = base[BASE_DAC];
				}
				break;

        	case mode_raw:
        		if(reg->tf.task.dacu <= base[BASE_DAC]){
					udac = reg->tf.task.dacu;
				}else{
					udac = base[BASE_DAC];
				}
        		if(reg->tf.task.daci <= base[BASE_DAC]){
					idac = reg->tf.task.daci;
				}else{
					idac = base[BASE_DAC];
				}
        		break;

        	case mode_off:
        	default:
        		idac = udac = 0;
        		if(reg->tf.state.bit.switchIsON != 0){
					l_switchRequest = setSwitchOff;
				}
				break;
        }

        /**************************************
         *Setting DAC value
         */
        setDacI(idac);
        setDacU(udac);
        //setDacU(iq_filtr(vTaskCumul, udac, VTASK_FILTER_K));

        if(reg->tf.task.mode == mode_overcurrentShutdown){
        	irqLimitOn();
        }
        else{
        	irqLimitOff();
        }

        /**************************************
         * Обработка запросов сохранения точки калибровки
         */
        if((reg->tf.task.request >= setSavePointU0)&&(reg->tf.task.request <= setSavePointU3)){
			s->pU[reg->tf.task.request - setSavePointU0].qu  = IntToIQ(reg->tf.task.u, 1000000);
			s->pU[reg->tf.task.request - setSavePointU0].adc = a->filtered[CH_UADC];
			s->pU[reg->tf.task.request - setSavePointU0].dac = reg->tf.task.dacu;
		}
        if((reg->tf.task.request >= setSavePointI0)&&(reg->tf.task.request <= setSavePointI3)){
			s->pI[reg->tf.task.request - setSavePointI0].qi  = IntToIQ(reg->tf.task.i, 1000000);
			s->pI[reg->tf.task.request - setSavePointI0].adc = a->filtered[CH_IADC];
			s->pI[reg->tf.task.request - setSavePointI0].dac = reg->tf.task.daci;

			s->pIEx[reg->tf.task.request - setSavePointI0].qi  = IntToIQ(reg->tf.task.i, 1000000);
			s->pIEx[reg->tf.task.request - setSavePointI0].adc = a->adcIna226;
			s->pIEx[reg->tf.task.request - setSavePointI0].dac = reg->tf.task.daci;
        }

		/**************************************
		 * Обработка запроса на сохранение калибровочной информации
		 */
		if(reg->tf.task.request == setSaveSettings){
			taskENTER_CRITICAL();
			prm_store(SYSFLASHADR, prmFlash);
			taskEXIT_CRITICAL();
			reg->tf.task.request = setNone;
		}

		/**************************************
		 * Выключение по времени
		 */
		if(reg->tf.task.mode == mode_timeShutdown){
			if((reg->tf.meas.time >= reg->tf.task.time)&&(reg->tf.state.bit.switchIsON != 0)){
				l_switchRequest = setSwitchOff;
			}
		}

		/**************************************
		 * Случай когда в режиме ограничения тока переключили на mode_limitation
		 */
		if(reg->tf.task.mode == mode_overcurrentShutdown){
			if(reg->tf.state.bit.modeIlim != 0){
				l_switchRequest = setSwitchOff;
			}
		}

		/**************************************
		 * Выключение по падению тока до 10% от задания
		 */
		if(reg->tf.task.mode == mode_lowCurrentShutdown){
			if(((reg->tf.meas.i <= (reg->tf.task.i / 10))&&(reg->tf.state.bit.switchIsON != 0))||(reg->tf.task.i == 0)){
				if((xTaskGetTickCount() - curOffTime) > pdMS_TO_TICKS(CUR_OFF_TIME)){
					l_switchRequest = setSwitchOff;
				}
			}else{
				curOffTime = xTaskGetTickCount();
			}
		}

		/**************************************
		 * При любых авариях выключаем выход
		 */
		if((reg->tf.state.bit.ovfLinearRegTemper != 0)||
		   (reg->tf.state.bit.errorLinearRegTemperSens != 0)||
		   (reg->tf.state.bit.lowInputVoltage != 0))
		{
			l_switchRequest = setSwitchOff;
		}

        /**************************************
         * Запросы на включение / отключение по каналу управления
         */
        if(reg->tf.task.request == setSwitchOn){
            l_switchRequest = setSwitchOn;
            reg->tf.task.request = setNone;
        }
        if(reg->tf.task.request == setSwitchOff){
            l_switchRequest = setSwitchOff;
            reg->tf.task.request = setNone;
        }

        /**************************************
         * Включение / отключение
         */
        if(l_switchRequest == setSwitchOn){
            if(reg->tf.state.bit.switchIsON == 0){
                reg->tf.state.bit.ovfCurrent = 0;
                setDacU(0);
                switchON();
            }
            timeOffset = xTaskGetTickCount();
            l_switchRequest = setNone;
        }
        if(l_switchRequest == setSwitchOff){
            if(reg->tf.state.bit.switchIsON != 0){
                setDacU(0);
                switchOFF();
            }
            l_switchRequest = setNone;
        }
    }
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

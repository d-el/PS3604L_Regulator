/*!****************************************************************************
 * @file		ds18TSK.c
 * @author		d_el
 * @version		V1.1
 * @date		06.04.2022
 * @brief
 * @copyright	The MIT License (MIT). Copyright (c) 2022 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <ds18b20.h>
#include <oneWireUart.h>
#include <crc.h>
#include <uart.h>
#include "ds18TSK.h"

/*!****************************************************************************
* MEMORY
*/
temperature_type temperature;

/*!****************************************************************************
* @brief
* @param
* @retval
*/
void ds18TSK(void *pPrm){
	(void)pPrm;
	uint8_t errorcnt = 0;

	ow_init();
	temperature.state = temp_Ok;

	/*****************************
	* DS18B20 INIT
	*/
	while(1){
		ds18b20state_type resInit = ds18b20Init(NULL);
		if(resInit == ds18b20st_ok){
			temperature.state = temp_Ok;
			break;
		}
		else{
			if(errorcnt < DS18_MAX_ERROR){
				errorcnt++;
			}else{
				temperature.state = temp_NoInit;
			}
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	while(1){
		vTaskDelay(pdMS_TO_TICKS(10));
		ds18b20ConvertTemp(NULL);

		vTaskDelay(pdMS_TO_TICKS(1000));

		uint8_t scratchpad[9];
		ds18b20state_type res = ds18b20ReadScratchpad(NULL, scratchpad);
		if(res != ds18b20st_ok)
			goto error;

		temperature.temperature = ds18b20Reg2tmpr(scratchpad[0], scratchpad[1]);
		temperature.state = temp_Ok;
		errorcnt = 0;

	continue;

	error:
			if(errorcnt < DS18_MAX_ERROR){
				errorcnt++;
			}else{
				temperature.state = temp_ErrSensor;
			}
	}
}

/******************************** END OF FILE ********************************/

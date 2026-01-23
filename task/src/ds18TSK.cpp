/*!****************************************************************************
 * @file		ds18TSK.c
 * @author		d_el
 * @version		V1.2
 * @date		30.03.2025
 * @brief
 * @copyright	The MIT License (MIT). Copyright (c) 2025 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include <assert.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <dev/ds18b20.h>
#include "driver/oneWireUart.h"
#include <crc.h>
#include <hal/uart.h>
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
	temperature.state = temp_NoInit;
	temperature.state = temp_Ok;

	/*****************************
	* DS18B20 INIT
	*/
	constexpr uint8_t convBits = 12;
	while(1){
		ds18b20_state_type resInit = ds18b20_init(NULL, convBits);
		if(resInit == ds18b20st_ok){
			temperature.state = temp_Ok;
			break;
		}
		else{
			if(errorcnt < DS18_MAX_ERROR){
				errorcnt++;
			}else{
				temperature.state = temp_ErrSensor;
			}
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	while(1){
		vTaskDelay(pdMS_TO_TICKS(10));
		ds18b20_convertTemp(NULL);

		vTaskDelay(pdMS_TO_TICKS(ds18b20_getTconv(convBits)));

		uint8_t scratchpad[9];
		ds18b20_state_type res = ds18b20_readScratchpad(NULL, scratchpad);
		if(res != ds18b20st_ok)
			goto error;

		temperature.temperature = ds18b20_reg2tmpr(scratchpad);
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

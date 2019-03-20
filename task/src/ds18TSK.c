/*!****************************************************************************
* @file    		ds18TSK.c
* @author  		d_el
* @version 		V1.0
* @date    		26.07.2016, Storozhenko Roman
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include <string.h>
#include "ds18b20.h"
#include "oneWireUart.h"
#include "crc.h"
#include "ds18TSK.h"
#include "uart.h"

/*!****************************************************************************
* MEMORY
*/
temperature_type   temperature;

/*!****************************************************************************
* @brief
* @param
* @retval
*/
void ds18TSK(void *pPrm){
    uint8_t errorcnt = 0;

    ow_init();
    temperature.state = temp_Ok;

    /*****************************
    * DS18B20 INIT
    */
    while(1){
    	ds18b20state_type resInit = ds18b20Init();
    	vTaskDelay(pdMS_TO_TICKS(1000));
        if(resInit == ds18b20st_ok){
        	temperature.state = temp_Ok;
        	break;
        }
        else{
        	temperature.state = temp_NoInit;
        }
    }

    while(1){
    	ow_setOutOpenDrain();
    	owSt_type st = ow_reset();
        if(st != owOk)
			goto error;

        uint8_t bff[9];
		bff[0] = SKIP_ROM;
		st = ow_write(bff, 1);
		if(st != owOk)
			goto error;

		bff[0] = READ_SCRATCHPAD;
		st = ow_write(bff, 1);
		if(st != owOk)
			goto error;

		st = ow_read(bff, 9);
		if(st != owOk)
			goto error;

		uint8_t crc = crc8Calc(&crc1Wire, bff, 9);
		if(crc != 0)
			goto error;

		uint16_t scratchpad = bff[1];
		scratchpad <<= 8;
		scratchpad |= bff[0];
		temperature.temperature = (scratchpad * 10 + (16/2)) / 16;   //Division with rounding
		temperature.state = temp_Ok;
		errorcnt = 0;

		st =  ow_reset();
		bff[0] = SKIP_ROM;
		ow_write(bff, 1);
		bff[0] = CONVERT_T;
		ow_write(bff, 1);

        ow_setOutHi();
        memset(bff, 0, 9);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;

        error:
			if(errorcnt < DS18_MAX_ERROR){
				errorcnt++;
			}else{
				temperature.state = temp_ErrSensor;
			}
	}
}


/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

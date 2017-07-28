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
#include "ds18TSK.h"

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
    uint16_t    scratchpad;
    uint8_t     bff[9];
    uint8_t     crc;
    uint8_t     resInit;
    uint8_t		errorcnt = 0;
    owSt_type   st;

    temperature.state = temp_Ok;

    /*****************************
    * DS18B20 INIT
    */
    //vTaskDelay(pdMS_TO_TICKS(10));
    while(1){
        resInit = ds18b20Init();
        if(resInit == 0){
        	temperature.state = temp_Ok;
        	break;
        }
        else{
        	temperature.state = temp_NoInit;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));	//Wait for converting

    while(1){
    	ow_setOutOpenDrain();
        st = ow_init();
        if(st == owOk){
            bff[0] = SKIP_ROM;
            ow_write(bff, 1);
            bff[0] = READ_SCRATCHPAD;
            ow_write(bff, 1);

            ow_read(bff, 9);
            crc = ow_crc8(bff, 9);

            if(crc == 0){
            	errorcnt = 0;
                scratchpad = bff[1];
                scratchpad <<= 8;
                scratchpad |= bff[0];
                temperature.temperature = (scratchpad * 10 + (16/2)) / 16;   //Деление с округлением
                temperature.state = temp_Ok;
            }
            else{
            	if(errorcnt < DS18_MAX_ERROR){
            		errorcnt++;
            	}else{
            		temperature.state = temp_ErrSensor;
            	}
            }

            st =  ow_init();
            bff[0] = SKIP_ROM;
            ow_write(bff, 1);
            bff[0] = CONVERT_T;
            ow_write(bff, 1);
        }
        else{
        	if(errorcnt < DS18_MAX_ERROR){
				errorcnt++;
			}else{
				temperature.state = temp_ErrSensor;
			}
        }

        ow_setOutHi();
        memset(bff, 0, 9);
        vTaskDelay(pdMS_TO_TICKS(1000));	//Wait for converting
    }
}


/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

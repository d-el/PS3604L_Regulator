/*!****************************************************************************
* @file			OSinit.c
* @author		D_EL - Storozhenko Roman
* @version      V1.0
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "assert.h"
#include "OSinit.h"
#include "systemTSK.h"

/*!****************************************************************************
* Semaphore
*/
SemaphoreHandle_t    AdcEndConversionSem;
SemaphoreHandle_t    i2c1Sem;

/*!****************************************************************************
*
*/
void OSinit(void){
    BaseType_t res;

    //AdcEndConversionSem
    vSemaphoreCreateBinary(AdcEndConversionSem);
    xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);
    //i2c1Sem
	vSemaphoreCreateBinary(i2c1Sem);
	xSemaphoreTake(i2c1Sem, portMAX_DELAY);

    res = xTaskCreate(systemTSK,    "systemTSK",    SYSTEM_TSK_SZ_STACK,    NULL, SYSTEM_TSK_PRIO,  NULL);
    assert(res == pdTRUE);

    vTaskStartScheduler();
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

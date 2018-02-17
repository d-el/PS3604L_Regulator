/*!****************************************************************************
* @file			OSinit.c
* @author		D_EL - Storozhenko Roman
* @version      V1.0
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "uartTSK.h"
#include "ds18TSK.h"
#include "OSinit.h"

/*!****************************************************************************
* Semaphore
*/
xSemaphoreHandle    AdcEndConversionSem;
xSemaphoreHandle    i2c1Sem;

/*!****************************************************************************
*
*/
void OSinit(void){
    BaseType_t Result = pdTRUE;;

    //AdcEndConversionSem
    vSemaphoreCreateBinary(AdcEndConversionSem);
    xSemaphoreTake(AdcEndConversionSem, portMAX_DELAY);
    //i2c1Sem
	vSemaphoreCreateBinary(i2c1Sem);
	xSemaphoreTake(i2c1Sem, portMAX_DELAY);


    Result &= xTaskCreate(uartTSK,      "uartTSK",      UART_TSK_SZ_STACK,      NULL, UART_TSK_PRIO,    NULL);
    Result &= xTaskCreate(adcTSK,       "adcTSK",       ADC_TSK_SZ_STACK,       NULL, ADC_TSK_PRIO,     NULL);
    Result &= xTaskCreate(ds18TSK,      "ds18TSK",      DS18B_TSK_SZ_STACK,     NULL, DS18B_TSK_PRIO,   NULL);
    Result &= xTaskCreate(systemTSK,    "systemTSK",    SYSTEM_TSK_SZ_STACK,    NULL, SYSTEM_TSK_PRIO,  NULL);

    if(Result == pdTRUE){
    }
    else{
        while(1);
    }

    vTaskStartScheduler();
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

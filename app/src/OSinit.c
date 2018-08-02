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

/*!****************************************************************************
*
*/
void OSinit(void){
    BaseType_t res;
    res = xTaskCreate(systemTSK,    "systemTSK",    SYSTEM_TSK_SZ_STACK,    NULL, SYSTEM_TSK_PRIO,  NULL);
    assert(res == pdTRUE);
    vTaskStartScheduler();
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

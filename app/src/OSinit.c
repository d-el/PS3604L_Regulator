/*!****************************************************************************
* @file			OSinit.c
* @author		D_EL - Storozhenko Roman
* @version      V1.0
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include <assert.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <systemTSK.h>

#define SYSTEM_TSK_SZ_STACK     512
#define SYSTEM_TSK_PRIO         3

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

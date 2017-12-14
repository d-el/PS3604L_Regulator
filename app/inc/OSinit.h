/*!****************************************************************************
* @file			OSinit.h
* @author		D_EL - Storozhenko Roman
* @version      V1.0
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "adcTSK.h"
#include "uartTSK.h"
#include "ds18TSK.h"
#include "systemTSK.h"

/*!****************************************************************************
*-------NAME--------------------size [2byte Word] */
#define SYSTEM_TSK_SZ_STACK     256
#define ADC_TSK_SZ_STACK        128
#define UART_TSK_SZ_STACK       128
#define DS18B_TSK_SZ_STACK      128
/******************************************************************************
*-------NAME--------------------size [2byte Word] */
#define SYSTEM_TSK_PRIO         3
#define ADC_TSK_PRIO            4
#define UART_TSK_PRIO           2
#define DS18B_TSK_PRIO          1

/*!****************************************************************************
* Mutex
*/
extern xSemaphoreHandle    oneWireBusyMutex;

/*!****************************************************************************
* Semaphore
*/
extern xSemaphoreHandle    AdcEndConversionSem;
extern xSemaphoreHandle    rxRequest;
extern xSemaphoreHandle    i2c1Sem;

/*!****************************************************************************
* Prototypes for the functions in OSinit.c
*/
void OSinit(void);

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

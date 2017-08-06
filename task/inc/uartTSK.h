/*!****************************************************************************
* @file    		uartTSK.h
* @author  		D_EL
* @version 		V1.1
* @date    		2015-20-11
* @copyright 	GNU Public License
*/
#ifndef uartTSK_H
#define uartTSK_H

/*!****************************************************************************
* Include
*/
#include "uart.h"
#include "crc.h"
#include "string.h"
#include "OSinit.h"

/*!****************************************************************************
* User define
*/
#define PIECE_BUF_RX          32	//[bytes]
#define maxWait_ms			1000
#define connectUart			(uart1)

/*!****************************************************************************
* User typedef
*/
typedef struct{
    uint16_t    waitRxTime_ms;
    uint16_t    readBytes;
    uint16_t    packTxCnt;
    uint16_t    packRxCnt;
    uint16_t    packErrorCnt;
}uartTask_type;

typedef enum{
    uartConnect,
    uartNoConnect
}uartTskState_type;

typedef struct{
	volatile uint32_t    		successfulRequest;
    volatile uint32_t    		errorRequest;
    volatile uartTskState_type	state;
}uartTsk_type;

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* Extern variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void uartTSK(void *pPrm);

#endif //uartTSK_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

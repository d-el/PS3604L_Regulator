/*!****************************************************************************
 * @file		uartTSK.h
 * @author		d_el
 * @version		V1.2
 * @date		13.12.2017
 * @brief		connect interface with regulator
 * @copyright	Copyright (C) 2017 Storozhenko Roman
 *				All rights reserved
 *				This software may be modified and distributed under the terms
 *				of the BSD license.	 See the LICENSE file for details
 */
#ifndef uartTSK_H
#define uartTSK_H

/*!****************************************************************************
* Include
*/
#include "stdint.h"

/*!****************************************************************************
* User define
*/
#define PIECE_BUF_RX        32	//[bytes]
#define maxWait_ms			1000
#define connectUart			uart1

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
* External variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void uartTSK(void *pPrm);

#endif //uartTSK_H
/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/

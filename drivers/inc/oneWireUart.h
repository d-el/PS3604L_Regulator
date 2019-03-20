/*!****************************************************************************
* @file    		oneWireUart.h
* @author  		d_el
* @version 		V1.0
* @date    		21.07.2016, Storozhenko Roman
* @copyright 	GNU Public License
*/
#ifndef oneWireUart_H
#define oneWireUart_H

/*!****************************************************************************
* Include
*/
#include "OSinit.h"
#include "semphr.h"

/*!****************************************************************************
* User define
*/
#define OW_UART     		(uart2)
#define OW_TIMEOUT     		(500)		//[ms]

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/
typedef enum{
    owOk,               //Устройство обнаружено
    owNotFound,
    owShortCircle,
	owTimeOut,
    owSearchLast,
    owSearchFinished,
    owSearchError,
	owUartTimeout
}owSt_type;

/*!****************************************************************************
* External variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void ow_init(void);
void ow_setOutHi(void);
void ow_setOutOpenDrain(void);
owSt_type ow_reset(void);
owSt_type ow_write(const void *src, uint8_t len);
owSt_type ow_read(void *dst, uint8_t len);
uint8_t ow_crc8(uint8_t *mas, uint8_t n);

#endif //oneWireUart_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

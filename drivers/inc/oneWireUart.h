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
#include "uart.h"
#include "OSinit.h"
#include "semphr.h"

/*!****************************************************************************
* User define
*/
#define OW_UART     		(uart3)
#define OW_TIMEOUT     		(100)		//[ms]

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/
typedef enum{
    owOk,               //Устройство обнаружено
    owNotFound,         //Не обнаружено
    owShortCircle,      //к.з. на линии
	owTimeOut,
    owSearchLast,       //Найден последний адрес
    owSearchFinished,   //Поиск остановлен, последний адрес был найден в предыдущий раз
    owSearchError
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
void ow_write(const void *src, uint8_t len);
void ow_read(void *dst, uint8_t len);
uint8_t ow_crc8(uint8_t *mas, uint8_t n);

#endif //oneWireUart_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

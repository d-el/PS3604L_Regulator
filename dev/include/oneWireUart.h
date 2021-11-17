/*!****************************************************************************
 * @file		oneWireUart.h
 * @author		d_el
 * @version		V1.0
 * @date		21.07.2016
 * @brief
 * @copyright	The MIT License (MIT). Copyright (c) 2021 Storozhenko Roman
 */
#ifndef oneWireUart_H
#define oneWireUart_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdint.h>

/*!****************************************************************************
* User define
*/
#define OW_UART				(uart2)
#define OW_TIMEOUT			(500)		//[ms]

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/
typedef enum{
	owOk,
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

#ifdef __cplusplus
}
#endif

#endif //oneWireUart_H
/******************************** END OF FILE ********************************/

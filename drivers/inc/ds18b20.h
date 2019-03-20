/*!****************************************************************************
* @file			ds18b20.h
* @author		d_el
* @version		V2.1
* @date			23.11.2015
* @copyright	GNU Public License
*/
#ifndef ds18b20_H
#define ds18b20_H

/*!****************************************************************************
* Include
*/
#include <stdint.h>
#include "oneWireUart.h"

/*!****************************************************************************
* User define
*/
//ROM COMMANDS Definition
#define SEARCH_ROM			0xF0
#define MATCH_ROM			0x55
#define READ_SCRATCHPAD		0xBE
#define READ_ROM			0x33
#define SKIP_ROM			0xCC
#define ALARM_SEARCH		0xEC
#define CONVERT_T			0x44
#define WRITE_SCRATCHPAD	0x4E
#define COPY_SCRATCHPAD		0x48
#define RECALL_E2			0xB8
#define READ_POWER_SUPPLY	0xB4

/*!****************************************************************************
* User typedef
*/
typedef struct{
	uint8_t		integer;
	uint8_t		frac;
	uint8_t		result;
	uint8_t		sign	:1;
	uint8_t		update	:1;
}tmpr_type;

typedef enum{
	ds18b20st_ok,
	ds18b20st_NotFound = owNotFound,
	ds18b20st_ShortCircle = owShortCircle,
	ds18b20st_wTimeOut = owTimeOut,
	ds18b20st_SearchLast = owSearchLast,
	ds18b20st_SearchFinished = owSearchFinished,
	ds18b20st_SearchError = owSearchError,
	ds18b20st_UartTimeout = owUartTimeout,
	ds18b20st_errorCrc,
	ds18b20st_notDs18b20
}ds18b20state_type;

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* External variables
*/
extern volatile		tmpr_type	tem;

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
uint8_t ds18b20Init(void);
uint16_t reg2tmpr(uint8_t rl, uint8_t rh);

#endif //ds18b20_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

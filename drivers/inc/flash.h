/*!****************************************************************************
* @file         flash.h
* @author		d_el
* @version      V2.1
* @date         07-02-2015
* @brief        flash driver
* @copyright 	GNU Public License
*/
#ifndef FLASH_H
#define FLASH_H

/*!****************************************************************************
* Include
*/
#include "stdint.h"

/*!****************************************************************************
* User define
*/

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/
typedef enum{
    flash_ok,
    flash_signatureError,
    flash_CRCError,
    flash_ErrorSizeMem,
    flash_error
}flashState_type;

/*!****************************************************************************
* External variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void flash_unlock(void);
void flash_lock(void);
void flash_eraseAllPages(void);
void flash_erasePage(void *address);
flashState_type flash_write(void *dst, uint16_t *src, uint32_t num);

#endif //FLASH_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

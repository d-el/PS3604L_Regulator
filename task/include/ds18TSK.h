/*!****************************************************************************
* @file    		ds18TSK.h
* @author  		d_el
* @version 		V1.0
* @date    		26.07.2016, Storozhenko Roman
* @copyright 	GNU Public License
*/
#ifndef ds18TSK_H
#define ds18TSK_H

/*!****************************************************************************
* Include
*/
#include <stdint.h>

/*!****************************************************************************
* User define
*/
#define DS18_MAX_ERROR		(3)	//Maximum error before set error state

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* User typedef
*/
typedef enum{
    temp_Ok,
	temp_NoInit,
    temp_ErrSensor
}temperatureState_type;

typedef struct{
    temperatureState_type   state;
    uint16_t                temperature;    //[X_X °C]
}temperature_type;

/*!****************************************************************************
* External variables
*/
extern temperature_type   temperature;

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void ds18TSK(void *pPrm);

#endif //ds18TSK_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

/*!****************************************************************************
* @file    		systemTSK.h
* @author  		Storozhenko Roman - D_EL
* @version 		V1.0
* @date    		14-09-2015
* @copyright 	GNU Public License
*/
#ifndef systemTSK_H
#define systemTSK_H

/*!****************************************************************************
* Include
*/
#include "pstypes.h"
#include "IQmathLib.h"

/*!****************************************************************************
* User define
*/
#define SYSTEM_TSK_PERIOD   (20)        ///<[ms]
#define CUR_OFF_TIME        (1000)      ///<[ms]
#define MAX_WAIT_RxRequest  (20)        ///<[ms]

#define COOLER_PWM_START    (0.3)       ///<[k PWM]
#define MIN_TEMP            (40.0)      ///<[°C]
#define MAX_TEMP            (60.0)      ///<[°C]
#define TEMP_OFF            (60.0)      ///<[°C]
#define H_TEMP              (5.0)       ///<[°C]

#define VTASK_FILTER_K      (3)

/*!****************************************************************************
* User typedef
*/
typedef struct{
    _iq         	qu;
    uint16_t     	adc;
    uint16_t     	dac;
}uCalibr_type;

typedef struct{
    _iq         	qi;
    uint16_t     	adc;
    uint16_t     	dac;
}iCalibr_type;

typedef struct{
    _iq         	qi;
    int16_t     	adc;
    uint16_t     	dac;
}iExtCalibr_type;

enum{
    BASE_DAC = 0,
};

typedef struct{
    uCalibr_type    pU[NUMBER_CALIBRATE_POINTS];
    iCalibr_type    pI[NUMBER_CALIBRATE_POINTS];
    iExtCalibr_type pIEx[NUMBER_CALIBRATE_POINTS];
}regSetting_type;

typedef struct{
    transfer_type           tf;
    regSetting_type         rgSet;
}regulator_type;

/*!****************************************************************************
* User enum
*/

/*!****************************************************************************
* External variables
*/
extern regulator_type      rg;

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void systemTSK(void *pPrm);

#endif //systemTSK_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

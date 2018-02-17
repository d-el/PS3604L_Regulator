/*!****************************************************************************
* @file    		ina226.h
* @author  		Storozhenko Roman - D_EL
* @version 		V1.0
* @date    		14.12.2016
* @copyright 	GNU Public License
*/

#ifndef INA226_H
#define INA226_H

/*!****************************************************************************
* Include
*/
#include "i2c.h"

/*!****************************************************************************
* User define
*/
#define ina226I2c		(i2c1)
#define ina226Addr		(0x80)
#define ina22_MAX_WAIT  (200)        ///<[ms]

/*!****************************************************************************
* User typedef
*/
enum{
    INA226_REG_CONFIGURATION = 0x00,
    INA226_REG_SHUNT_VOLTAGE = 0x01,
    INA226_REG_BUS_VOLTAGE = 0x02,
    INA226_REG_POWER = 0x03,
    INA226_REG_CURRENT = 0x04,
    INA226_REG_CALIBRATION = 0x05,
    INA226_REG_MASK_ENABLE = 0x06,
    INA226_REG_ALERT_LIMIT = 0x07,
    INA226_REG_DIE_ID = 0xFF,
};

enum{
    INA226_CONFIGURATION__RST = 0x8000,

    INA226_CONFIGURATION__AVG_MASK = 0x0E00,
    INA226_CONFIGURATION__AVG_SHIFT = 9,

    INA226_CONFIGURATION__BUS_CONV_TIME_MASK = 0x01C0,
    INA226_CONFIGURATION__BUS_CONV_TIME_SHIFT = 6,

    INA226_CONFIGURATION__SHUNT_CONV_TIME_MASK = 0x0038,
    INA226_CONFIGURATION__SHUNT_CONV_TIME_SHIFT = 3,

    INA226_CONFIGURATION__MODE_MASK = 0x0007,
    INA226_CONFIGURATION__MODE_SHIFT = 0,
};

enum{
    INA226_MASK_ENABLE__CVRF = 0x0008,
};

enum{
	INA226_DIE_ID_REG_VAL = 0x2260
};

/**
 * Possible sampling periods.
 */
typedef enum{
    INA226_PERIOD_140us = 0,
    INA226_PERIOD_204us = 1,
    INA226_PERIOD_332us = 2,
    INA226_PERIOD_588us = 3,
    INA226_PERIOD_1100us = 4,
    INA226_PERIOD_2116us = 5,
    INA226_PERIOD_4156us = 6,
    INA226_PERIOD_8244us = 7,
} ina226_sampling_period_t;

/** Possible averaging values */
typedef enum{
    INA226_AVERAGE_1 = 0,
    INA226_AVERAGE_4 = 1,
    INA226_AVERAGE_16 = 2,
    INA226_AVERAGE_64 = 3,
    INA226_AVERAGE_128 = 4,
    INA226_AVERAGE_256 = 5,
    INA226_AVERAGE_512 = 6,
    INA226_AVERAGE_1024 = 7,
} ina226_averaging_factor_t;

typedef enum{
	INA226_MODE_Power_Down = 0,
	INA226_MODE_ShuntVoltageTriggered = 1,
	INA226_MODE_BusVoltageTriggered = 2,
	INA226_MODE_ShuntAndBusTriggered = 3,
	PINA226_MODE_Power_Down = 4,
	INA226_MODE_ShuntVoltageContinuous = 5,
	INA226_MODE_BusVoltageContinuous = 6,
	INA226_MODE_ShuntAndBusContinuous = 7,
}ina226_mode_t;

typedef enum{
	INA226_OK,
	INA226_ERROR
}ina226State_type;

/*!****************************************************************************
* External variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
ina226State_type ina226_readReg(uint16_t *dst, uint8_t addr);
ina226State_type ina226_writeReg(uint16_t value, uint8_t addr);
ina226State_type ina226_configure(ina226_sampling_period_t period, ina226_averaging_factor_t average, ina226_mode_t mode);
ina226State_type ina226_trig(void);
ina226State_type ina226_readShuntVoltage(int16_t *dst);
ina226State_type ina226_verifyConnect(void);

#endif //INA226_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

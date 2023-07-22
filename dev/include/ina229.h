/*!****************************************************************************
 * @file    	ina226.h
 * @author  	Storozhenko Roman - D_EL
 * @version 	V1.0
 * @date    	14.12.2016
 * @copyright 	The MIT License (MIT). Copyright (c) 2020 Storozhenko Roman
 */
#ifndef INA226_H
#define INA226_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
* Include
*/
#include <stdbool.h>

/*!****************************************************************************
* User define
*/
#define ina226I2c		(i2c1)
#define ina226Addr		(0x80)
#define ina22_MAX_WAIT  (200)        ///<[ms]

/*!****************************************************************************
* User typedef
*/
typedef enum{
	CONFIG = 0x00,          // Configuration 16
	ADC_CONFIG = 0x01,      // ADC Configuration 16
	SHUNT_CAL = 0x02,       // Shunt Calibration 16
	SHUNT_TEMPCO = 0x03,    // Shunt Temperature Coefficient 16
	VSHUNT = 0x04,          // Shunt Voltage Measurement 24
	VBUS = 0x05,            // Bus Voltage Measurement 24
	DIETEMP = 0x06,         // Temperature Measurement 16
	CURRENT = 0x07,         // Current Result 24
	POWER = 0x08,           // Power Result 24
	ENERGY = 0x09,          // Energy Result 40
	CHARGE = 0x0A,          // Charge Result 40
	DIAG_ALRT = 0x0B,       // Diagnostic Flags and Alert 16
	SOVL = 0x0C,            // Shunt Overvoltage Threshold 16
	SUVL = 0x0D,            // Shunt Undervoltage Threshold 16
	BOVL = 0x0E,            // Bus Overvoltage Threshold 16
	BUVL = 0x0F,            // Bus Undervoltage Threshold 16
	TEMP_LIMIT = 0x10,      // Temperature Over-Limit Threshold 16
	PWR_LIMIT = 0x11,       // Power Over-Limit Threshold 16
	MANUFACTURER_ID = 0x3E,	// Manufacturer ID 16
	DEVICE_ID = 0x3F		// ID Device ID 16	1
}ina229_registers_t;

/*!****************************************************************************
* External variables
*/

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
bool ina229_init(void);
bool ina229_trig(void);
bool ina229_readShuntVoltage(int32_t *v);
bool ina229_readCNVRF(bool *c);

#ifdef __cplusplus
}
#endif

#endif //INA226_H
/******************************** END OF FILE ********************************/

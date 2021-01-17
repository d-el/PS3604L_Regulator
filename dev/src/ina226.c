/*!****************************************************************************
* @file    		ina226.c
* @author  		Storozhenko Roman - D_EL
* @version 		V1.0
* @date    		14.12.2016
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include "ina226.h"

/*!****************************************************************************
* MEMORY
*/
SemaphoreHandle_t i2c1Sem;

/*!****************************************************************************
 * @brief	I2C callback
 */
static void i2c1TC_Hook(i2c_type *i2cx){
	(void)i2cx;
    BaseType_t  xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(i2c1Sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken != pdFALSE){
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
}

/*!****************************************************************************
* @brief
* @retval
*/
ina226State_type ina226_init(void){
	vSemaphoreCreateBinary(i2c1Sem);
	xSemaphoreTake(i2c1Sem, portMAX_DELAY);

	i2c_setCallback(i2c1, i2c1TC_Hook);

	return INA226_OK;
}

/*!****************************************************************************
* @brief
* @retval
*/
ina226State_type ina226_readReg(uint16_t *dst, uint8_t addr){
	BaseType_t		osRes;
	uint8_t 		buf[2];

    // Write the register address
    buf[0] = addr;
    i2c_write(ina226I2c, buf, 1, ina226Addr, i2cNeedStop);
    osRes = xSemaphoreTake(i2c1Sem, pdMS_TO_TICKS(ina22_MAX_WAIT));
    if(osRes != pdTRUE){
    	return INA226_ERROR;
    }

    // Read the value
    i2c_read(ina226I2c, buf, 2, ina226Addr);
    osRes = xSemaphoreTake(i2c1Sem, pdMS_TO_TICKS(ina22_MAX_WAIT));
	if(osRes != pdTRUE){
		return INA226_ERROR;
	}

    *dst = (buf[0] << 8) + buf[1];
    return INA226_OK;
}

/*!****************************************************************************
* @brief
* @retval
*/
ina226State_type ina226_writeReg(uint16_t value, uint8_t addr){
	BaseType_t		osRes;
	uint8_t 		buf[3];

	buf[0] = addr;
	buf[1] = value >> 8;
	buf[2] = value & 0xFF;

    // Write the register address
    i2c_write(ina226I2c, buf, 3, ina226Addr, i2cNeedStop);
    osRes = xSemaphoreTake(i2c1Sem, pdMS_TO_TICKS(ina22_MAX_WAIT));
    if(osRes != pdTRUE){
    	return INA226_ERROR;
    }

    return INA226_OK;
}

/*!****************************************************************************
* @brief
* @retval
*/
ina226State_type ina226_configure(ina226_sampling_period_t period, ina226_averaging_factor_t average, ina226_mode_t mode){
    // Prepare register value;
    uint16_t reg;
    reg = (period << INA226_CONFIGURATION__BUS_CONV_TIME_SHIFT)
            & INA226_CONFIGURATION__BUS_CONV_TIME_MASK;
    reg |= (period << INA226_CONFIGURATION__SHUNT_CONV_TIME_SHIFT)
            & INA226_CONFIGURATION__SHUNT_CONV_TIME_MASK;
    reg |= (average << INA226_CONFIGURATION__AVG_SHIFT)
            & INA226_CONFIGURATION__AVG_MASK;
    reg |= mode & INA226_CONFIGURATION__MODE_MASK;

    // Write the configuration value
    return ina226_writeReg(reg, INA226_REG_CONFIGURATION);
}

/*!****************************************************************************
* @brief
* @retval
*/
ina226State_type ina226_trig(void){
	return ina226_configure(INA226_PERIOD_4156us, INA226_AVERAGE_16, INA226_MODE_ShuntVoltageTriggered);
}

/*!****************************************************************************
* @brief
* @retval
*/
ina226State_type ina226_readShuntVoltage(int16_t *dst){
	return ina226_readReg((uint16_t*)dst, INA226_REG_SHUNT_VOLTAGE);
}

/*!****************************************************************************
* @brief
* @retval
*/
ina226State_type ina226_verifyConnect(void){
	uint16_t reg = 0;

	ina226_readReg(&reg, INA226_REG_DIE_ID);
	if(reg == INA226_DIE_ID_REG_VAL){
		return INA226_OK;
	}else{
		return INA226_ERROR;
	}

}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

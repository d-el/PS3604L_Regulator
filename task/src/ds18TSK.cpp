/*!****************************************************************************
 * @file		ds18TSK.c
 * @author		d_el
 * @version		V1.2
 * @date		30.03.2025
 * @brief
 * @copyright	The MIT License (MIT). Copyright (c) 2025 Storozhenko Roman
 */

/*!****************************************************************************
* Include
*/
#include <string.h>
#include <assert.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <ds18b20.h>
#include <crc.h>
#include <uart.h>
#include "oneWireUart.h"
#include "ds18TSK.h"

#define OW_UART		(uart2)

/*!****************************************************************************
* MEMORY
*/
temperature_type temperature;
static SemaphoreHandle_t oneWireUartSem;

/*!****************************************************************************
 * @brief	uart RX callback
 */
static void uartRxHook(uart_type *puart){
	(void)puart;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(oneWireUartSem, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/*!****************************************************************************
* @brief
* @param
* @retval
*/
void ds18TSK(void *pPrm){
	(void)pPrm;
	uint8_t errorcnt = 0;
	temperature.state = temp_NoInit;

	// Create Semaphore for UART
	vSemaphoreCreateBinary(oneWireUartSem);
	assert(oneWireUartSem != NULL);
	xSemaphoreTake(oneWireUartSem, portMAX_DELAY);
	uart_setCallback(OW_UART, (uartCallback_type)NULL, uartRxHook);
	auto uartInit = []() -> bool {
		uart_init(OW_UART, 9600);	//1WIRE
		return true;
	};
	auto uartSetBaud = [](uint32_t baud) -> bool {
		uart_setBaud(OW_UART, baud);
		return true;
	};
	auto uartWrite = [](const void* src, size_t len) -> bool {
		uart_write(OW_UART, src, len);
		return true;
	};
	static
	auto uartReadEnable = [](void* dst, size_t len) -> size_t {
		uart_read(OW_UART, dst, len);
		return true;
	};
	auto uartRead = [](void* dst, size_t len) -> size_t {
		(void)dst;
		BaseType_t res = xSemaphoreTake(oneWireUartSem, pdMS_TO_TICKS(50));
		if(res != pdTRUE){
			return 0;
		}
		return len - uartGetRemainRx(OW_UART);
	};

	// Init one wire stack
	ow_init(uartInit,
			uartSetBaud,
			uartWrite,
			uartReadEnable,
			uartRead,
			nullptr);
	temperature.state = temp_Ok;

	/*****************************
	* DS18B20 INIT
	*/
	constexpr uint8_t convBits = 12;
	while(1){
		ds18b20_state_type resInit = ds18b20_init(NULL, convBits);
		if(resInit == ds18b20st_ok){
			temperature.state = temp_Ok;
			break;
		}
		else{
			if(errorcnt < DS18_MAX_ERROR){
				errorcnt++;
			}else{
				temperature.state = temp_ErrSensor;
			}
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	while(1){
		vTaskDelay(pdMS_TO_TICKS(10));
		ds18b20_convertTemp(NULL);

		vTaskDelay(pdMS_TO_TICKS(ds18b20_getTconv(convBits)));

		uint8_t scratchpad[9];
		ds18b20_state_type res = ds18b20_readScratchpad(NULL, scratchpad);
		if(res != ds18b20st_ok)
			goto error;

		temperature.temperature = ds18b20_reg2tmpr(scratchpad);
		temperature.state = temp_Ok;
		errorcnt = 0;

	continue;

	error:
			if(errorcnt < DS18_MAX_ERROR){
				errorcnt++;
			}else{
				temperature.state = temp_ErrSensor;
			}
	}
}

/******************************** END OF FILE ********************************/

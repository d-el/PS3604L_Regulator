/*!****************************************************************************
 * @file		oneWireUart.cpp
 * @author		d_el
 * @version		V1.0
 * @date		22.01.2026
 * @copyright	License (MIT). Copyright (c) 2026 Storozhenko Roman
 * @brief
 */

/*!****************************************************************************
 * Include
 */
#include <assert.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <crc.h>
#include <hal/uart.h>
#include "oneWireUart.h"

#ifndef OW_RESETBAUD
#define OW_RESETBAUD 9600
#endif
#ifndef OW_RWBITBAUD
#define OW_RWBITBAUD 115200
#endif

// ROM COMMANDS Definition
#define SEARCH_ROM			0xF0
#define MATCH_ROM			0x55
#define READ_ROM			0x33
#define SKIP_ROM			0xCC
#define ALARM_SEARCH		0xEC

static OneWire oneWire0;
static SemaphoreHandle_t oneWireUartSem;
#define OW_UART		(uart2)

/*!****************************************************************************
* @brief	Initialization one wire interface
*/
bool OneWire::init(){
	// Create Semaphore for UART
	oneWireUartSem = xSemaphoreCreateBinary();
	assert(oneWireUartSem != NULL);

	auto uartRxHook = [](uart_type *puart){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR((SemaphoreHandle_t)puart->rxHoockArg, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	};
	uart_setCallback(OW_UART, (uartCallback_type)NULL, NULL, uartRxHook, oneWireUartSem);

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

	m_owuart_init = uartInit;
	m_owuart_setBaud = uartSetBaud;
	m_owuart_write = uartWrite;
	m_owuart_readEnable = uartReadEnable;
	m_owuart_read = uartRead;
	m_owuart_strongPullup = nullptr;

	return m_owuart_init();
}

/*!****************************************************************************
* @brief	Set strong pullup
* @param	true - pullup enable, false - pullup disable
* @retval	owSt_t
*/
OneWire::owSt_t OneWire::strongPullup(bool v){
	if(m_owuart_strongPullup){
		m_owuart_strongPullup(v);
		return owOk;
	}
	return owOther;
}

/*!****************************************************************************
* @brief	Reset pulse and presence detect
* @param	None
* @retval	owSt_t
*/
OneWire::owSt_t OneWire::reset(void){
	m_owuart_setBaud(OW_RESETBAUD);
	m_owuart_readEnable(rxBff, sizeof(rxBff));
	txBff[0] = 0xF0;
	m_owuart_write(txBff, 1);
	size_t len = m_owuart_read(rxBff, sizeof(rxBff));
	if(len < 1){
		return owUartTimeout;
	}
	if((rxBff[0] >= 0x90)&&(rxBff[0] <= 0xF0)){
		return owOk;
	}else{
		return owNotFound;
	}
}

/*!***************************************************************************
* @brief	Write one bit
* @param	src - bit value
* @retval	None
*/
OneWire::owSt_t OneWire::writebit(uint8_t src){
	uint8_t *pBff = txBff;
	uint8_t byteTrans = 1;
	if(src != 0){
		*pBff = 0xFF;
	}else{
		*pBff = 0x00;
	}
	m_owuart_setBaud(OW_RWBITBAUD);
	m_owuart_readEnable(rxBff, byteTrans);
	m_owuart_write(txBff, byteTrans);
	size_t len = m_owuart_read(rxBff, byteTrans);
	if(len < 1){
		return OneWire::owUartTimeout;
	}
	return OneWire::owOk;
}

/*!***************************************************************************
* @brief	Write data
* @param	src		pointer to source buffer
* @param	len		number bytes for transmit
* @retval	None
*/
OneWire::owSt_t OneWire::write(const void *src, uint8_t len){
	uint8_t *pSrc		= (uint8_t*)src;
	uint8_t *pSrcEnd	= pSrc + len;
	uint8_t *pBff		= txBff;
	uint8_t mask, byteTrans = len << 3;

	while(pSrc < pSrcEnd){
		for(mask = 1; mask != 0; mask <<= 1){
			if((*pSrc & mask) != 0){
				*pBff++ = 0xFF;
			}else{
				*pBff++ = 0x00;
			}
		}
		pSrc++;
	}

	m_owuart_setBaud(OW_RWBITBAUD);
	m_owuart_readEnable(rxBff, byteTrans);
	m_owuart_write(txBff, byteTrans);
	size_t rxlen = m_owuart_read(rxBff, byteTrans);
	if(rxlen < 1){
		return owUartTimeout;
	}
	return owOk;
}

/*!***************************************************************************
* @brief	Read one bit
* @param	dst - pointer to destination bt value
* @retval	Status operation
*/
OneWire::owSt_t OneWire::readbit(uint8_t *dst){
	uint8_t	 byteTrans = 1;
	memset(txBff, 0xFF, byteTrans);
	m_owuart_setBaud(OW_RWBITBAUD);
	m_owuart_readEnable(rxBff, byteTrans);
	m_owuart_write(txBff, byteTrans);
	size_t len = m_owuart_read(rxBff, byteTrans);
	if(len > 0){
		if(rxBff[0] == 0xFF){
			*dst = 1; //Read '1'
		}else{
			*dst = 0;
		}
	}
	else{
		return OneWire::owUartTimeout;
	}

	return OneWire::owOk;
}

/*!***************************************************************************
* @brief	Read data
* @param	dst - pointer to destination buffer
* @param	len - number bytes for receive
* @retval	Status operation
*/
OneWire::owSt_t OneWire::read(void *dst, uint8_t len){
	uint8_t		*pDst		= (uint8_t*)dst;
	uint8_t		*pDstEnd	= pDst + len;
	uint8_t		*pBff		= rxBff;
	uint8_t		mask, byteTrans = len << 3;

	memset(txBff, 0xFF, byteTrans);

	m_owuart_setBaud(OW_RWBITBAUD);
	m_owuart_readEnable(rxBff, byteTrans);
	m_owuart_write(txBff, byteTrans);
	size_t rxlen = m_owuart_read(rxBff, byteTrans);
	if(rxlen == byteTrans){
		while(pDst < pDstEnd){
			*pDst = 0;
			for(mask = 1; mask != 0; mask <<= 1){
				if(*pBff++ == 0xFF){
					*pDst |= mask; //Read '1'
				}
			}
			pDst++;
		}
	}
	else{
		return OneWire::owUartTimeout;
	}

	return OneWire::owOk;
}

/*!***************************************************************************
 * @brief	Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing search state
 * @param	Searching context
 * @retval	Status operation
 */
OneWire::owSt_t OneWire::searchRom(uint8_t rom[8]){
	owSt_t searchResult = owOk;
	uint8_t lastZero = 0, idBitNumber = 1, romByteNumber = 0;
	uint8_t idBit, cmpIdBit;
	uint8_t romByteMask = 1, searchDirection;

	// If the last call was not the last one
	if(!lastDeviceFlag){
		if(reset() != OneWire::owOk){
			// Reset the search
			lastDiscrepancy = 0;
			lastDeviceFlag = 0;
			lastFamilyDiscrepancy = 0;
			return OneWire::owNotFound;
		}

		// Issue the search command
		const uint8_t search = SEARCH_ROM;
		write(&search, 1);
		// loop to do the search
		do{
			// Read a bit and its complement
			readbit(&idBit);
			readbit(&cmpIdBit);

			// Check for no devices on 1-wire
			if((idBit == 1) && (cmpIdBit == 1)){
				break;
			}else{
				// All devices coupled have 0 or 1
				if(idBit != cmpIdBit){
					searchDirection = idBit; // bit write value for search
				}else{
					// If this discrepancy if before the Last Discrepancy
					// On a previous next then pick the same as last time
					if(idBitNumber < lastDiscrepancy){
						searchDirection = ((rom[romByteNumber] & romByteMask) > 0);
					}else{
						// If equal to last pick 1, if not then pick 0
						searchDirection = (idBitNumber == lastDiscrepancy);
					}
					// If 0 was picked then record its position in LastZero
					if(searchDirection == 0){
						lastZero = idBitNumber;
						// Check for Last discrepancy in family
						if(lastZero < 9)
							lastFamilyDiscrepancy = lastZero;
					}
				}
				// Set or clear the bit in the ROM byte rom_byte_number
				// With mask rom_byte_mask
				if(searchDirection == 1){
					rom[romByteNumber] |= romByteMask;
				}else{
					rom[romByteNumber] &= ~romByteMask;
				}
				// Serial number search direction write bit
				writebit(searchDirection);
				// Increment the byte counter id_bit_number
				// And shift the mask rom_byte_mask
				idBitNumber++;
				romByteMask <<= 1;
				// If the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if(romByteMask == 0){
					romByteNumber++;
					romByteMask = 1;
				}
			}
		}while(romByteNumber < 8); // Loop until through all ROM bytes 0-7
		// If the search was successful then
		if(!((idBitNumber < 65) || (crc8Calc(&crc1Wire, rom, 8) != 0))){
			// Search successful so set lastDiscrepancy, lastDeviceFlag, search_result
			lastDiscrepancy = lastZero;
			// Check for last device
			if(lastDiscrepancy == 0){
				lastDeviceFlag = 1;
				searchResult = OneWire::owSearchLast;
			}else{
				searchResult = OneWire::owSearchOk;
			}
		}
	}
	// If no device found then reset counters so next 'search' will be like a first
	if(!searchResult || rom[0] == 0){
		lastDiscrepancy = 0;
		lastDeviceFlag = 0;
		lastFamilyDiscrepancy = 0;
		searchResult = OneWire::owSearchError;
	}
	return searchResult;
}

/*!***************************************************************************
 * @brief	Read the slave’s 64-bit ROM code
 * @param	rom		destination
 * @retval	Status operation
 */
OneWire::owSt_t OneWire::readRom(uint8_t rom[8]){
	uint8_t buff[8];
	buff[0] = READ_ROM;
	owSt_t result = write(buff, 1);
	if(result != OneWire::owOk){
		return result;
	}

	result = read(rom, 8);
	if(result != OneWire::owOk){
		return result;
	}

	uint8_t crc = crc8Calc(&crc1Wire, rom, 8);
	if(crc != 0){
		return OneWire::owCrcError;
	}
	return OneWire::owOk;
}

/*!***************************************************************************
 * @brief	Address a specific slave device
 * @param	rom - slave ID or NULL for skip ROM
 * @retval	Status operation
 */
OneWire::owSt_t OneWire::selectRom(const uint8_t rom[8]){
	uint8_t buff[9];
	uint8_t size = 0;

	if(rom != NULL){
		buff[size++] = MATCH_ROM;
		for(size_t i = 0; i < 8; i++){
			buff[size++] = rom[i];
		}
	}else{
		buff[size++] = SKIP_ROM;
	}
	OneWire::owSt_t result = write(buff, size);
	return result;
}

OneWire& oneWire(uint8_t number){
	(void)number;
	return oneWire0;
}

/******************************** END OF FILE ********************************/

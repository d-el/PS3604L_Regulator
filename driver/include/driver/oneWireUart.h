/*!****************************************************************************
 * @file		oneWireUart.h
 * @author		d_el
 * @version		V1.0
 * @date		22.01.2026
 * @copyright	License (MIT). Copyright (c) 2026 Storozhenko Roman
 * @brief
 */

#ifndef oneWireUart_H
#define oneWireUart_H

/*!****************************************************************************
 * Include
 */
#include <stdint.h>
#include <stdint.h>

#ifndef OW_TX_BFF
#define OW_TX_BFF 128
#endif

#ifndef OW_RX_BFF
#define OW_RX_BFF 128
#endif

class OneWire{
public:
	using owSt_t = enum{
		owOk,
		owNotFound,
		owShortCircle,
		owTimeOut,
		owCrcError,
		owSearchOk,
		owSearchLast,
		owSearchError,
		owUartTimeout,
		owOther
	};

	typedef bool (*owuart_init_t)(void);
	typedef bool (*owuart_strongPullup_t)(bool hi);
	typedef bool (*owuart_setBaud_t)(uint32_t baud);
	typedef bool (*owuart_write_t)(const void* src, size_t len);
	typedef size_t (*owuart_readEnable_t)(void* dst, size_t len);
	typedef size_t (*owuart_read_t)(void* dst, size_t len);

public:
	bool init();
	owSt_t strongPullup(bool v);
	owSt_t reset(void);
	owSt_t write(const void *src, uint8_t len);
	owSt_t read(void *dst, uint8_t len);
	owSt_t searchRom(uint8_t rom[8]);
	owSt_t readRom(uint8_t rom[8]);
	owSt_t selectRom(const uint8_t rom[8]);

private:
	owSt_t writebit(uint8_t src);
	owSt_t readbit(uint8_t *dst);

private:
	owuart_init_t m_owuart_init;
	owuart_setBaud_t m_owuart_setBaud;
	owuart_write_t m_owuart_write;
	owuart_readEnable_t m_owuart_readEnable;
	owuart_read_t m_owuart_read;
	owuart_strongPullup_t m_owuart_strongPullup;

	uint8_t txBff[OW_TX_BFF];
	uint8_t rxBff[OW_RX_BFF];
	uint8_t rom[8];
	uint8_t lastDiscrepancy;
	uint8_t lastFamilyDiscrepancy;
	uint8_t lastDeviceFlag;
};

OneWire& oneWire(uint8_t number);

#endif //oneWireUart_H
/******************************** END OF FILE ********************************/

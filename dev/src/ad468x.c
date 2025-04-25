/*!****************************************************************************
 * @file		ad468x.c
 * @author		d_el
 * @version		V1.0
 * @date		Dec 24, 2024
 * @copyright	License (MIT). Copyright (c) 2024 Storozhenko Roman
 * @brief
 */

/*!****************************************************************************
 * Include
 */
#include <stddef.h>
#include "ad468x.h"
#include "gpio.h"

enum{
	N_CONFIGURATION1 = 0x01,
	N_CONFIGURATION2,
	N_ALERT,
	N_ALERT_LOW_THRESHOLD,
	N_ALERT_HIGH_THRESHOLD
};

enum{
	READ = 0,
	WRITE = 1
};

typedef union{
	struct{
		uint16_t PMODE		:1;
		uint16_t REFSEL		:1;
		uint16_t RES		:1;
		uint16_t ALERT_EN	:1;
		uint16_t CRC_R		:1;
		uint16_t CRC_W		:1;
		uint16_t OSR		:3;
		uint16_t OS_MODE	:1;
		uint16_t RESERVED	:2;
		uint16_t REGADDR	:3;
		uint16_t WR			:1;
	}bit;
	uint16_t all;
}CONFIGURATION1_t;

typedef union{
	struct{
		uint16_t RESET		:8;
		uint16_t SDO		:1;
		uint16_t RESERVED	:3;
		uint16_t REGADDR	:3;
		uint16_t WR			:1;
	}bit;
	uint16_t all;
}CONFIGURATION2_t;

typedef union{
	struct{
		uint16_t AL_A_LOW	:1;
		uint16_t AL_A_HIGH	:1;
		uint16_t RESERVED	:2;
		uint16_t AL_B_LOW	:1;
		uint16_t AL_B_HIGH	:1;
		uint16_t RESERVED1	:2;
		uint16_t SETUP_F	:1;
		uint16_t CRCW_F		:1;
		uint16_t RESERVED2	:2;
		uint16_t REGADDR	:3;
		uint16_t WR			:1;
	}bit;
	uint16_t all;
}ALERT_t;

typedef union{
	struct{
		uint16_t ALERT_LOW	:12;
		uint16_t REGADDR	:3;
		uint16_t WR			:1;
	}bit;
	uint16_t all;
}ALERT_LOW_THRESHOLD_t;

typedef union{
	struct{
		uint16_t ALERT_HIGH	:12;
		uint16_t REGADDR	:3;
		uint16_t WR			:1;
	}bit;
	uint16_t all;
}ALERT_HIGH_THRESHOLD_t;

/*!****************************************************************************
 * MEMORY
 */
static ad468x_spi_t spi;

/*!****************************************************************************
 * @brief
 */
static bool ad468x_tr(const void *src, void *dst, uint16_t len){
	uint8_t rx[16] = {};
	uint8_t tx[16] = {};

	if(src){
		const uint8_t *w = (uint8_t*)src;
		for(size_t i = 0; i < len; i++){
			tx[i] = w[len - i - 1];
		}
	}
	gppin_reset(GP_ADC_NSS);
	bool res = spi(rx, tx, len);
	gppin_set(GP_ADC_NSS);

	if(dst){
		uint8_t *r = (uint8_t*)dst;
		for(size_t i = 0; i < len; i++){
			r[i] = rx[len - i - 1];
		}
	}

	return res;
}

/*!****************************************************************************
 * @brief
 */
void ad468x_init(ad468x_spi_t _spi){
	spi = _spi;

	CONFIGURATION1_t cfg1;
	cfg1.all = 0;
	cfg1.bit.WR = WRITE;
	cfg1.bit.REGADDR = N_CONFIGURATION1;
	cfg1.bit.OS_MODE = 1;					// Oversampling Mode enable
	cfg1.bit.OSR = 3;						// Oversampling Ratio 8x
	cfg1.bit.RES = 1;						// 2-bit higher resolution
	cfg1.bit.REFSEL = 1;					// selects external reference
	ad468x_tr(&cfg1, NULL, 2);

	CONFIGURATION2_t cfg2;
	cfg2.all = 0;
	cfg2.bit.WR = WRITE;
	cfg2.bit.REGADDR = N_CONFIGURATION2;
	cfg2.bit.SDO = 1; 						// 1-wire, conversion data are output on SDOA only
	ad468x_tr(&cfg2, NULL, 2);
}

/*!****************************************************************************
 * @brief
 */
bool ad468x_convRead(int32_t *result_a, int32_t *result_b){
	uint64_t regs = 0;
	ad468x_tr(NULL, &regs, 5);
	*result_a = (regs & 0xFFFFC00000UL) >> 22;
	*result_b = (regs & 0x00003FFFF0UL) >> 4;
	return true;
}

/******************************** END OF FILE ********************************/

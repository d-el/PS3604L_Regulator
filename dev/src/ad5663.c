/*!****************************************************************************
 * @file		ad5663.c
 * @author		d_el
 * @version		V1.0
 * @date		Mar 26, 2024
 * @copyright	License (MIT). Copyright (c) 2024 Storozhenko Roman
 * @brief
 */

/*!****************************************************************************
 * Include
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <spi.h>
#include <gpio.h>
#include "ad5663.h"
#include "ina229.h"

typedef struct __attribute__((packed)){
	union{
		struct{
			uint8_t address	:3;
			uint8_t command	:3;
			uint8_t res		:2;
			}bit;
		uint8_t all;
	};
	uint16_t db;
}ad5663pack_type;

enum{
	write_to_input_register_n = 0,
	update_DAC_register_n,
	write_to_input_register_n_update_all,
	write_to_and_update_DAC_channel_n,
	power_down_DAC,
	reset,
	LDAC_register_setup,
	reserved
};

enum{
	DAC_A = 0,
	DAC_B,
	reserved0,
	reserved1,
	reserved2,
	reserved3,
	reserved4,
	all_DACs
};

/*!****************************************************************************
 * MEMORY
 */
static ad5663_spi_t spi;

/*!****************************************************************************
 * @brief
 */
static bool ad5663_writepack(ad5663pack_type p){
	uint8_t rx[3] = {};
	gppin_reset(GP_AD5663_SYNC);
	bool res = spi(rx, (void*)&p, 3);
	gppin_set(GP_AD5663_SYNC);
	return res;
}

/*!****************************************************************************
 * @brief
 */
void ad5663_init(ad5663_spi_t _spi){
	spi = _spi;
	ad5663_writepack((ad5663pack_type){ .bit.command = reset, .db = 1 }); // Reset
	ad5663_writepack((ad5663pack_type){ .bit.command = power_down_DAC, .db = 3 }); // Normal operation
}

/*!****************************************************************************
 * @brief
 */
void ad5663_set_a(uint16_t val){
	ad5663_writepack((ad5663pack_type){ .bit.command = write_to_and_update_DAC_channel_n, .bit.address = DAC_A, .db = val >> 8 | val << 8 });
}

/*!****************************************************************************
 * @brief
 */
void ad5663_set_b(uint16_t val){
	ad5663_writepack((ad5663pack_type){ .bit.command = write_to_and_update_DAC_channel_n, .bit.address = DAC_B, .db = val >> 8 | val << 8 });
}

/******************************** END OF FILE ********************************/

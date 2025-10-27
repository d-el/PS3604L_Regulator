/*!****************************************************************************
 * @file		ad5060.h
 * @author		d_el
 * @version		V1.0
 * @date		Oct 23, 2025
 * @copyright	License (MIT). Copyright (c) 2025 Storozhenko Roman
 * @brief
 */

#ifndef ad5060_H
#define ad5060_H

#ifdef __cplusplus
extern "C" {
#endif


/*!****************************************************************************
 * Include
 */
#include <stdint.h>
#include <stdbool.h>

typedef bool (*ad5060_spi_t)(void* dst, const void* src, uint16_t len);

/*!****************************************************************************
 * Function declaration
 */
void ad5060_init(ad5060_spi_t spi);
void ad5060_set_a(uint16_t val);
void ad5060_set_b(uint16_t val);

#ifdef __cplusplus
}
#endif

#endif //ad5663_H
/******************************** END OF FILE ********************************/

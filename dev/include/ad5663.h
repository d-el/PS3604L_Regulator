/*!****************************************************************************
 * @file		ad5663.h
 * @author		d_el
 * @version		V1.0
 * @date		Mar 26, 2024
 * @copyright	License (MIT). Copyright (c) 2024 Storozhenko Roman
 * @brief
 */

#ifndef ad5663_H
#define ad5663_H

#ifdef __cplusplus
extern "C" {
#endif


/*!****************************************************************************
 * Include
 */
#include <stdbool.h>

typedef bool (*ad5663_spi_t)(void* dst, const void* src, uint16_t len);

/*!****************************************************************************
 * Function declaration
 */
void ad5663_init(ad5663_spi_t spi);
void ad5663_set_a(uint16_t val);
void ad5663_set_b(uint16_t val);

#ifdef __cplusplus
}
#endif

#endif //ad5663_H
/******************************** END OF FILE ********************************/

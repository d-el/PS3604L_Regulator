/*!****************************************************************************
 * @file		ad468x.h
 * @author		d_el
 * @version		V1.0
 * @date		Dec 24, 2024
 * @copyright	License (MIT). Copyright (c) 2024 Storozhenko Roman
 * @brief
 */

#ifndef ad468x_H
#define ad468x_H

#ifdef __cplusplus
extern "C" {
#endif

/*!****************************************************************************
 * Include
 */
#include <stdint.h>
#include <stdbool.h>

/*!****************************************************************************
 * Define
 */

/*!****************************************************************************
 * Enumeration
 */

/*!****************************************************************************
 * Typedef
 */
typedef bool (*ad468x_spi_t)(void* dst, const void* src, uint16_t len);

/*!****************************************************************************
 * Exported variables
 */

/*!****************************************************************************
 * Macro functions
 */

/*!****************************************************************************
 * Function declaration
 */
void ad468x_init(ad468x_spi_t spi);
bool ad468x_convRead(int32_t *result_a, int32_t *result_b);

#ifdef __cplusplus
}
#endif

#endif //ad468x_H
/******************************** END OF FILE ********************************/

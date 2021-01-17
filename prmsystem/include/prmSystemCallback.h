/*!****************************************************************************
 * @file		prmSystemCallback.h
 * @author		d_el
 * @version		V1.0
 * @date		Dec 11, 2019
 * @brief
 */

#ifndef prmSystemCallback_H
#define prmSystemCallback_H

/*!****************************************************************************
 * Include
 */
#include "prmSystem.h"

/*!****************************************************************************
 * Define
 */

/*!****************************************************************************
 * Enumeration
 */

/*!****************************************************************************
 * Typedef
 */

/*!****************************************************************************
 * Exported variables
 */

/*!****************************************************************************
 * Macro functions
 */

/*!****************************************************************************
 * Function declaration
 */
void pcpy(const struct prmHandle* h, void *arg);
void getFwVer(const struct prmHandle* h, void *arg);

void vsave(const struct prmHandle* h, void *arg);
void isave(const struct prmHandle* h, void *arg);

#endif //prmSystemCallback_H
/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/

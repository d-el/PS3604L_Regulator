/*!****************************************************************************
 * @file		prmsystem.h
 * @author		d_el
 * @version		V1.0
 * @date		May 26, 2020
 * @brief
 * @copyright	Copyright (C) 2017 Storozhenko Roman
 *				All rights reserved
 *				This software may be modified and distributed under the terms
 *				of the BSD license.	 See the LICENSE file for details
 */

#ifndef prmsystem_H
#define prmsystem_H

#include <stdint.h>

class IPrm {
public:
	virtual ~IPrm() = default;

	virtual bool operator==( const IPrm& value ) const = 0;

	virtual bool operator!=( const IPrm& value ) const {
		return !( *this == value );
	}
};

class Int16Prm final : public IPrm {
	using Int16Type = uint16_t;

private:
	Int16Type m_value = 0;
};

#endif //prmsystem_H
/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/

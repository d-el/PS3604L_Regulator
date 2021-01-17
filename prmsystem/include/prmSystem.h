/*!****************************************************************************
 * @file		prmSystem.h
 * @author		d_el - Storozhenko Roman
 * @version		V1.0
 * @date		05.11.2020
 * @copyright	The MIT License (MIT). Copyright (c) 2017 Storozhenko Roman
 * @brief		Parameters system
 */
#ifndef PRMSYSTEM_H
#define PRMSYSTEM_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/*!****************************************************************************
 * Include
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*!****************************************************************************
 * Define
 */
#define typeboolFrmt	bool
#define typecharFrmt	char
#define typeu8Frmt		uint8_t
#define	types8Frmt		int8_t
#define	typeu16Frmt		uint16_t
#define	types16Frmt		int16_t
#define	typeu32Frmt		uint32_t
#define	types32Frmt		int32_t
#define	typefloatFrmt	float

#define	power0			1
#define	power1			10
#define	power2			100
#define	power3			1000U
#define	power4			10000U
#define	power5			100000U
#define	power6			1000000U

typedef uint32_t t_unixTimeFrmt;
typedef uint32_t t_unixDateFrmt;
typedef uint32_t t_ipAdrFrmt;

#include <parametrestype.h>

/*!****************************************************************************
 * Typedef
 */
typedef union {
	bool		t_boolFrmt;
	char		t_charFrmt;
	uint8_t		t_u8Frmt;
	int8_t		t_s8Frmt;
	uint16_t	t_u16Frmt;
	int16_t		t_s16Frmt;
	uint32_t	t_u32Frmt;
	int32_t		t_s32Frmt;
	float		t_floatFrmt;
	uint32_t	t_unixTimeFrmt;
	uint32_t	t_unixDateFrmt;
	uint32_t	t_ipAdrFrmt;
	uint8_t		bytes[4];
} prmval_type;

typedef enum {
	boolFrmt,
	charFrmt,
	u8Frmt,
	s8Frmt,
	u16Frmt,
	s16Frmt,
	u32Frmt,
	s32Frmt,
	floatFrmt,
	unixTimeFrmt,
	unixDateFrmt,
	ipAdrFrmt,
	bytesFmt
} prmType_type;

typedef enum {
	chmodR,
	chmodRW
} prmChmod_type;

typedef enum {
	prmNotSave,
	prmSaveSys,
	prmSaveUser
} prmNvSave_type;

typedef enum {
	prmLimConst,
	prmLimVariable
} prmLim_type;

typedef enum {
	prmStepConst,
	prmStepVariable
} prmStep_type;

typedef enum {
	prm_ok,
	prm_addrIsNull,
	prm_signatureError,
	prm_crcError,
	prm_errorSizeMem,
	prm_writeError,
	prm_readError,
	prm_error
} prm_state_type;

typedef struct prmHandle{
	prmval_type		*prm;			//Pointer to parameter
	prmval_type		def;
	prmval_type		min;
	prmval_type		max;
	const char		*label;
	const char		*units;
	void (*pCallback)(const struct prmHandle*, void *arg);
	uint16_t		addr;
	uint16_t		parameter;
	uint8_t			power		:4;
	prmType_type	type		:4;
	prmChmod_type	chmod		:2;
	prmNvSave_type	save		:2;
} prmHandle_type;

/*!****************************************************************************
 * Exported variables
 */

/*!****************************************************************************
 * Macro functions
 */

/*!****************************************************************************
 * Function declaration
 */
bool prm_init(void);
uint8_t prm_getSize(const prmHandle_type *const prmHandle);
const prmHandle_type *prm_getHandler(uint16_t index);
void prm_callback(const prmHandle_type *prmHandle, void *arg);
void prm_writeVal(const prmHandle_type *const prmHandle, const prmval_type prmval, void *arg);
prmval_type prm_readVal(const prmHandle_type *const prmHandle);
prmval_type prm_nreadVal(parametresNum_type parametres);
void prm_loadDefault(prmNvSave_type prmNvSave);
size_t prm_size(prmNvSave_type prmNvSave);
prm_state_type prm_serialize(void *pMemory, size_t *size, prmNvSave_type prmNvSave);
prm_state_type prm_deserialize(void *pMemory, prmNvSave_type prmNvSave);
size_t prm_toString(char *string, size_t size, const prmHandle_type *prmHandler);
void prm_toPrm(const char *string, const prmHandle_type *prmHandler);
bool prm_greaterThan(const prmHandle_type *const ph, prmval_type a, prmval_type b);
bool prm_lessThan(const prmHandle_type *const ph, prmval_type a, prmval_type b);
bool prm_equal(const prmHandle_type *const ph, prmval_type a, prmval_type b);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PRMSYSTEM_H */
/******************************** END OF FILE ********************************/

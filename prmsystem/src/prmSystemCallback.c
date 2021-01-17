/*!****************************************************************************
 * @file		prmSystemCallback.c
 * @author		d_el
 * @version		V1.0
 * @date		Dec 11, 2019
 * @brief
 */

/*!****************************************************************************
 * Include
 */
#include <string.h>
#include <stddef.h>
#include <version.h>
#include "prmSystemCallback.h"

/*!****************************************************************************
 * @brief
 */
void pcpy(const struct prmHandle* h, void *arg){
	(void)arg;
	if(h == NULL){
		return;
	}
	const prmHandle_type *p = prm_getHandler(h->parameter);
	if(p == NULL){
		return;
	}
	if(h->type != p->type){
		return;
	}
	switch(h->type){
		case boolFrmt:
			p->prm->t_boolFrmt = h->prm->t_boolFrmt;
			break;
		case charFrmt:
			p->prm->t_charFrmt = h->prm->t_charFrmt;
			break;
		case u8Frmt:
			p->prm->t_u8Frmt = h->prm->t_u8Frmt;
			break;
		case s8Frmt:
			p->prm->t_s8Frmt = h->prm->t_s8Frmt;
			break;
		case u16Frmt:
			p->prm->t_u16Frmt = h->prm->t_u16Frmt;
			break;
		case s16Frmt:
			p->prm->t_s16Frmt = h->prm->t_s16Frmt;
			break;
		case u32Frmt:
			p->prm->t_u32Frmt = h->prm->t_u32Frmt;
			break;
		case s32Frmt:
			p->prm->t_s32Frmt = h->prm->t_s32Frmt;
			break;
		case floatFrmt:
			p->prm->t_floatFrmt = h->prm->t_floatFrmt;
			break;
		case unixTimeFrmt ... bytesFmt:
			p->prm->t_floatFrmt = h->prm->t_floatFrmt;
			break;
	}
}

/*!****************************************************************************
 * @brief
 */
void getFwVer(const struct prmHandle* h, void *arg){
	(void)arg;
	if(h->parameter == 0){
		h->prm->t_u16Frmt = fwinfoMajor;
	}else if(h->parameter == 1){
		h->prm->t_u16Frmt = fwinfoMinor;
	}else if(h->parameter == 2){
		h->prm->t_u16Frmt = fwinfoRevision;
	}else if(h->parameter == 3){
		h->prm->t_u16Frmt = fwinfoBuild;
	}
}

/***************** Copyright (C) Storozhenko Roman ******* END OF FILE *******/

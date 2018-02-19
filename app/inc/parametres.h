/*!****************************************************************************
 * @file		parametres.h
 * @author		d_el - Storozhenko Roman
 * @version		V1.0
 * @date		08.02.2017
 * @copyright	GNU Lesser General Public License v3
 * @brief		Parameters table
 */

//			label		,units,prm					,type			,chmod			,def			,min			,max		,step	,bigst	,pow,limtype		,steptype		,savetype

//Point U
parametres(su0u		,""		,rg.sett.pU[0].qu		,s32Frmt		,chmodAlways	,_IQ(0.0101)	,_IQ(0.00000)	,_IQ(36)	,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su0adc	,""		,rg.sett.pU[0].adc		,u16Frmt		,chmodAlways	,414			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su0dac	,""		,rg.sett.pU[0].dac		,u16Frmt		,chmodAlways	,25				,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(su1u		,""		,rg.sett.pU[1].qu		,s32Frmt		,chmodAlways	,_IQ(0.1050)	,_IQ(0.00000)	,_IQ(36)	,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su1adc	,""		,rg.sett.pU[1].adc		,u16Frmt		,chmodAlways	,580			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su1dac	,""		,rg.sett.pU[1].dac		,u16Frmt		,chmodAlways	,35				,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(su2u		,""		,rg.sett.pU[2].qu		,s32Frmt		,chmodAlways	,_IQ(19.0000)	,_IQ(0.00000)	,_IQ(36)	,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su2adc	,""		,rg.sett.pU[2].adc		,u16Frmt		,chmodAlways	,33230			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su2dac	,""		,rg.sett.pU[2].dac		,u16Frmt		,chmodAlways	,2021			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(su3u		,""		,rg.sett.pU[3].qu		,s32Frmt		,chmodAlways	,_IQ(30.0000)	,_IQ(0.00000)	,_IQ(36)	,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su3adc	,""		,rg.sett.pU[3].adc		,u16Frmt		,chmodAlways	,52292			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(su3dac	,""		,rg.sett.pU[3].dac		,u16Frmt		,chmodAlways	,3178			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

//Point I
parametres(si0i		,""		,rg.sett.pI[0].qi		,s32Frmt		,chmodAlways	,_IQ(0.00966)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si0adc	,""		,rg.sett.pI[0].adc		,u16Frmt		,chmodAlways	,562			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si0dac	,""		,rg.sett.pI[0].dac		,u16Frmt		,chmodAlways	,35				,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(si1i		,""		,rg.sett.pI[1].qi		,s32Frmt		,chmodAlways	,_IQ(0.01007)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si1adc	,""		,rg.sett.pI[1].adc		,u16Frmt		,chmodAlways	,1980			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si1dac	,""		,rg.sett.pI[1].dac		,u16Frmt		,chmodAlways	,121			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(si2i		,""		,rg.sett.pI[2].qi		,s32Frmt		,chmodAlways	,_IQ(1.5000)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si2adc	,""		,rg.sett.pI[2].adc		,u16Frmt		,chmodAlways	,24539			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si2dac	,""		,rg.sett.pI[2].dac		,u16Frmt		,chmodAlways	,1491			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(si3i		,""		,rg.sett.pI[3].qi		,s32Frmt		,chmodAlways	,_IQ(3.0000)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si3adc	,""		,rg.sett.pI[3].adc		,u16Frmt		,chmodAlways	,48647			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(si3dac	,""		,rg.sett.pI[3].dac		,u16Frmt		,chmodAlways	,2958			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

//Point I from external ADC
parametres(sei0i	,""		,rg.sett.pIEx[0].qi		,s32Frmt		,chmodAlways	,_IQ(0.00966)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei0adc	,""		,rg.sett.pIEx[0].adc	,u16Frmt		,chmodAlways	,50				,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei0dac	,""		,rg.sett.pIEx[0].dac	,u16Frmt		,chmodAlways	,35				,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(sei1i	,""		,rg.sett.pIEx[1].qi		,s32Frmt		,chmodAlways	,_IQ(0.01007)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei1adc	,""		,rg.sett.pIEx[1].adc	,u16Frmt		,chmodAlways	,340			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei1dac	,""		,rg.sett.pIEx[1].dac	,u16Frmt		,chmodAlways	,121			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(sei2i	,""		,rg.sett.pIEx[2].qi		,s32Frmt		,chmodAlways	,_IQ(1.5000)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei2adc	,""		,rg.sett.pIEx[2].adc	,u16Frmt		,chmodAlways	,3214			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei2dac	,""		,rg.sett.pIEx[2].dac	,u16Frmt		,chmodAlways	,1491			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

parametres(sei3i	,""		,rg.sett.pIEx[3].qi		,s32Frmt		,chmodAlways	,_IQ(3.0000)	,_IQ(0.00000)	,_IQ(4)		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei3adc	,""		,rg.sett.pIEx[3].adc	,u16Frmt		,chmodAlways	,32767			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)
parametres(sei3dac	,""		,rg.sett.pIEx[3].dac	,u16Frmt		,chmodAlways	,2958			,0000			,0xFFFF		,1		,1		,0	,prmLimConst	,prmStepConst	,prmFlash)

/*************** LGPL ************** END OF FILE *********** D_EL ************/

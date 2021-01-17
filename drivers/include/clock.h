/*!****************************************************************************
* @file    		clock.h
* @author  		d_el
* @version 		V1.0
* @date    		09.01.2016, by d_el
* @copyright 	GNU Public License
*/
#ifndef clock_H
#define clock_H

/*!****************************************************************************
* Include
*/
#include "stdint.h"

/*!****************************************************************************
* User define
*/
#define RCC_CRYSTAL_OSCILLATOR_FREQ     24000000
#define RCC_RC_OSCILLATOR_FREQ          8000000
#define HSE_PLL_MUL                     2           //2-16, Коэффициент умножения PLL
#define HSE_PLL_PREDIV                  2           //1-16, Коэффициент деления PLL
#define HSI_PLL_MUL                     6           //2-16, Коэффициент умножения PLL
#define HSI_PLL_PREDIV                  1           //1-16, Коэффициент деления PLL
#define RCC_WAIN_TIMEOUT                100000

/*!****************************************************************************
* User enum
*/
typedef enum{
    clock_useHsi,
	clock_useHse
}useGen_type;

/*!****************************************************************************
* User typedef
*/
typedef struct{
    uint32_t        currentSysFrec;
    useGen_type     useGen;
}clock_type;

/*!****************************************************************************
* External variables
*/
extern clock_type clockSource;

/*!****************************************************************************
* Macro functions
*/

/*!****************************************************************************
* Prototypes for the functions
*/
void clock_init(void);

#endif //clock_H
/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

/*!****************************************************************************
* @file         flash.c
* @author       d_el
* @version      V2.1
* @date         07.02.2015
* @copyright 	GNU Public License
*/

/*!****************************************************************************
* Include
*/
#include "flash.h"

/*!****************************************************************************
* Memory
*/
uint8_t nvMemTmpBff[210];
nvMem_memreg_type   nvMem_memreg[] = {
    { &rg.rgSet,      sizeof(regSetting_type)      },
};

nvMem_struct_type   nvMem = {
    sizeof(nvMem_memreg) / sizeof(nvMem_memreg_type),
    nvMem_memreg,
    nvMemBaseAdr(flashPage1),
    nvMemBaseAdr(flashPage2),
};

/*!****************************************************************************
* @brief    Erase all pages
*/
void flash_eraseAllPages(void){
    FLASH->CR |= FLASH_CR_MER;
    FLASH->CR |= FLASH_CR_STRT;
    while(flash_busy());
    FLASH->CR &= FLASH_CR_MER;
}

/*!****************************************************************************
* @brief    	Erase one page
* @param[in]    addr - address allocable in page
*/
void flash_erasePage(void *addr){
    FLASH->CR|= FLASH_CR_PER;
    FLASH->AR = (uint32_t)addr;
    FLASH->CR|= FLASH_CR_STRT;
    while(flash_busy());
    FLASH->CR&= ~FLASH_CR_PER;
}

/*!****************************************************************************
* @brief    Write data
* @param    dst[in] - destination
* @param    src[in] - source
* @param    num[in] - number half word (2 byte)
*/
nvMem_state_type flash_write(void *dst, void *src, uint32_t num){
    uint16_t    *pRd;
    uint16_t    *pWr;
    uint16_t    *pEnd;

    flash_unlock();

    FLASH->CR |= FLASH_CR_PG;
    while(flash_busy());

    pRd     = src;
    pWr     = dst;
    pEnd    = pWr + num;
    
    while(pWr < pEnd){
        *pWr++ = *pRd++;
        while(flash_busy());
    }

    FLASH->CR &= ~FLASH_CR_PG;
    flash_lock();
    return nvMem_ok;
}

/*!****************************************************************************
* @brief    Init system
*/
nvMem_state_type nvMem_init(void){
    nvMem.fullSize  = 0;
    
    for(uint16_t i = 0; i < nvMem.numPrm; i++){
        nvMem.fullSize += nvMem.memreg[i].sizeofData;
    }
    nvMem.fullSize += sizeof(nvMem.saveCnt);
    nvMem.fullSize += sizeof(uint16_t);         //signature
    nvMem.fullSize += sizeof(uint16_t);         //CRC
    if(nvMem.fullSize > sizeof(nvMemTmpBff)){
        while(1){
        	__NOP();
        }
    }
    return nvMem_ok;
}

/*!****************************************************************************
* @brief    save data
* @param    adrNvMem[in]
*/
nvMem_state_type nvMem_savePrm(void *adrNvMem){
    uint8_t     *ptr;
    
    ptr   = nvMemTmpBff;
    
    //Check flag
    if(nvMem.flags.bit.prepareForWrite == 0){
        nvMem_prepareMemory(adrNvMem);
    }

    //Increment counter
    nvMem.saveCnt++;

    //Concatenate data
    *(uint16_t*)ptr = nvMemSignature;		//Signature
    ptr += sizeof(uint16_t);
    //saveCnt
    *(uint16_t*)ptr = nvMem.saveCnt;
    ptr += sizeof(nvMem.saveCnt);
    //memreg
    for(uint16_t i = 0; i < nvMem.numPrm; i++){
        memcpy(ptr, nvMem.memreg[i].dataPtr, nvMem.memreg[i].sizeofData);
        ptr += nvMem.memreg[i].sizeofData;
    }
    //CRC
    *(uint16_t*)ptr = GetCrc(nvMemTmpBff, ptr - nvMemTmpBff);
    ptr += sizeof(uint16_t);
    
    //Sava
    flash_write(adrNvMem, nvMemTmpBff, ptr - nvMemTmpBff);
    
    nvMem.flags.bit.prepareForWrite = 0;
    
    return nvMem_ok;
}

/*!****************************************************************************
* @brief
* @param[in]    adrNvMem - Р°РґСЂРµСЃ РёСЃС‚РѕС‡РЅРёРєР° РІ flash РїР°РјСЏС‚Рё
* @retval   - nvMem_ok
*           - nvMem_CRCError
*           - nvMem_signatureError
*/
nvMem_state_type nvMem_loadPrm(void *adrNvMem){
    uint8_t     *ptr;
    uint16_t    l_crc, l_signature;
    
    ptr   = nvMemTmpBff;
    
    memcpy(nvMemTmpBff,  adrNvMem, nvMem.fullSize);
    
    l_crc = GetCrc(nvMemTmpBff, nvMem.fullSize);
    
    if(l_crc == 0){ //CRC OK
        l_signature = *(uint16_t*)ptr;
        if(l_signature != nvMemSignature){  //signature error
            return nvMem_signatureError;
        }
        ptr += sizeof(l_signature);
        nvMem.saveCnt = *(uint16_t*)ptr;
        ptr += sizeof(nvMem.saveCnt);
        
        for(uint16_t i = 0; i < nvMem.numPrm; i++){
            memcpy(nvMem.memreg[i].dataPtr, ptr, nvMem.memreg[i].sizeofData);
            ptr += nvMem.memreg[i].sizeofData;
        }
        
        return nvMem_ok;
    }
    else{   //CRC ERROR
        return nvMem_CRCError;
    }
}

/*!****************************************************************************
*
*/
nvMem_state_type nvMem_prepareMemory(void *adrNvMem){
    flash_unlock();
    flash_erasePage(adrNvMem);
    flash_lock();
    
    nvMem.flags.bit.prepareForWrite = 1;
    
    return nvMem_ok;
}

/*************** GNU GPL ************** END OF FILE ********* D_EL ***********/

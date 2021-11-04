# PS3604LR

### Overview
This repository include the source code of the regulator power Supply PS3604L.  
MCU: STM32F373CCT6:  
    - LQFP48, ARM Cortex-M4 32b MCU+FPU, up to 256KB Flash + 32KB SRAM, timers,  
    - 4 ADCs (16-bit Sig. Delta / 12-bit SAR), 3 DACs, 2 comp., 2.0-3.6 V  

###Requirements
toolchain arm-none-eabi 10.3 or higher  
gcc / g++ 7.5.0 or higher  
cmake 3.14 or higher  
libjsoncpp-dev  

###Build project
>mkdir build  
>cd build  
>cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake  
>make -j8  

#### Start debug server
>./scripts/gdb_serv_start.sh openocd_mculink

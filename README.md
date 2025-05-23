# PS3604LR

<p float="left">
  <img src="image/IMG_4709.JPG" width="200" />
  <img src="image/IMG_4719.JPG" width="200" /> 
  <img src="image/IMG_E4582.JPG" width="200" />
</p>

### Overview
This repository include the source code of the linear regulator Power Supply [PS3604L](https://github.com/d-el/PS3604L).  
MCU: STM32F373CCT6:  
    - LQFP48, ARM Cortex-M4 32b MCU+FPU, up to 256KB Flash + 32KB SRAM, timers,  
    - 4 ADCs (16-bit Sig. Delta / 12-bit SAR), 3 DACs, 2 comp., 2.0-3.6 V  

### Requirements
toolchain arm-none-eabi 10.3 or higher  
gcc / g++ 7.5.0 or higher  
cmake 3.14 or higher  
libjsoncpp-dev  
crc32  

### Build project
>mkdir build  
>cd build  
>cmake .. -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake  
>make -j8  

#### Start debug server
>./scripts/gdb-serv.sh openocd-jlink

#### Start debug client
>./scripts/gdb-client.sh

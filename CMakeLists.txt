cmake_minimum_required(VERSION 3.6)

PROJECT(PS3604LR CXX C ASM)
set(CMAKE_CXX_STANDARD 17)

SET(CMAKE_CROSSCOMPILING TRUE)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
set(CMAKE_C_COMPILER    arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER  arm-none-eabi-g++)
set(CMAKE_OBJCOPY		arm-none-eabi-objcopy)
set(CMAKE_SIZE			arm-none-eabi-size)

set(AR                  arm-none-eabi-ar)
set(OBJCOPY             arm-none-eabi-objcopy)
set(OBJDUMP             arm-none-eabi-objdump)
set(SIZE                arm-none-eabi-size)
set(GDB                 arm-none-eabi-gdb)
set(SIZE                arm-none-eabi-size)

set(CPUFLAGS "\
	-mcpu=cortex-m4 \
	-march=armv7e-m \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16 \
	")

set(COMMONFLAGS "\
	-g3 -O2 \
	-fmessage-length=0 \
	-ffunction-sections \
	-fdata-sections \
	-fsigned-char \
	-fsingle-precision-constant \
	-Wfloat-equal \
	-Wuninitialized \
	-Wextra \
	-Wall \
	")

add_compile_options(
	-DSTM32F373xC
	)

set(CMAKE_CXX_FLAGS "${CPUFLAGS} ${COMMONFLAGS} -std=c++17 -fno-rtti -fno-exceptions")
set(CMAKE_C_FLAGS "${CPUFLAGS} ${COMMONFLAGS} -std=gnu11")

set(FUSED "\
	-u _IQ14mpy \
	-u _IQ14div \
	")

set(CMAKE_EXE_LINKER_FLAGS "\
	${CPUFLAGS} \
	${FUSED} \
	-T ${CMAKE_CURRENT_SOURCE_DIR}/systems/STM32F373CC_FLASH.ld \
	-Wl,--gc-sections \
	-Xlinker --gc-sections --specs=nano.specs \
	-Wl,--print-memory-usage \
	-Wl,--undefined=uxTopUsedPriority \
	-Wl,-Map=${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.map \
	${CMAKE_CURRENT_SOURCE_DIR}/lib/IQmathLib-cm4/IQmathLib-cm4.a \
	 ")

add_subdirectory(config)
add_subdirectory(dev)
add_subdirectory(drivers)
add_subdirectory(lib/CMSIS)
add_subdirectory(lib/freemodbus)
add_subdirectory(lib/freertos)
add_subdirectory(lib/IQmathLib-cm4)
add_subdirectory(math)
add_subdirectory(portfreemodbus)
add_subdirectory(prmsystem)
add_subdirectory(systems)
add_subdirectory(task)
add_subdirectory(utils)

set(SOURCE_FILES
	app/src/main.c
	app/src/OSinit.c
	)

add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})

include_directories(PUBLIC
	app/include
	)

target_link_libraries(${PROJECT_NAME}.elf
	drivers
	freertos
	systems
	task
	utils
	)

# Additional command which will run after the above from a different directory
add_custom_command(TARGET ${PROJECT_NAME}.elf PRE_BUILD
	COMMAND ${PROJECT_SOURCE_DIR}/scripts/vgen.sh utils/src/version.c
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
)

set(HEX_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_SOURCE_DIR}/build/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
	COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
	COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}.elf>
	COMMENT "Building ${HEX_FILE} \nBuilding ${BIN_FILE}"
	)

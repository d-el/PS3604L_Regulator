cmake_minimum_required(VERSION 3.6)

PROJECT(PS3604LR
	VERSION 1.12.2
	LANGUAGES CXX C
	)
	
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(
	-DSTM32F373xC
	)

add_compile_options(
	-g3 -O2
	-fno-exceptions
	-fmessage-length=0
	-ffunction-sections
	-fdata-sections
	-fsigned-char
	-fsingle-precision-constant
	-Wfloat-equal
	-Wuninitialized
	-Wextra
	-Wall

	$<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
	)

add_link_options(
	-Wl,--print-memory-usage
	-Wl,--undefined=uxTopUsedPriority
	-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
	)

configure_file(utils/include/version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/utils/include/version.h @ONLY)

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

include_directories(PUBLIC
	app/include
	)

add_executable(${PROJECT_NAME}.elf main.cpp)
target_link_options(${PROJECT_NAME}.elf
	PRIVATE
	-Wl,--defsym=CALC_CRC=0x00000000
	-Wl,--defsym=IMAGE_SIZE=00000000
	-Wl,--defsym=V_MAJOR=${PROJECT_VERSION_MAJOR}
	-Wl,--defsym=V_MINOR=${PROJECT_VERSION_MINOR}
	-Wl,--defsym=V_PATCH=${PROJECT_VERSION_PATCH}
	-L ${CMAKE_CURRENT_SOURCE_DIR}/systems
	-T main.ld
	)
target_link_libraries(${PROJECT_NAME}.elf
	drivers
	freertos
	systems
	task
	utils
	${CMAKE_CURRENT_SOURCE_DIR}/lib/IQmathLib-cm4/IQmathLib-cm4.a
	)

add_executable(${PROJECT_NAME}b.elf boot.cpp)
target_link_options(${PROJECT_NAME}b.elf
	PRIVATE
	-Wl,--defsym=CALC_CRC=0x00000000
	-L ${CMAKE_CURRENT_SOURCE_DIR}/systems
	-T boot.ld
	)
target_link_libraries(${PROJECT_NAME}b.elf
	drivers
	freertos
	systems
	task
	utils
	${CMAKE_CURRENT_SOURCE_DIR}/lib/IQmathLib-cm4/IQmathLib-cm4.a
	)

set(NAME ${PROJECT_NAME}_${PROJECT_VERSION})
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
	# Link executable with CRC
	BYPRODUCTS ${PROJECT_NAME}.temp.bin
	COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.temp.bin --gap-fill=0xFF # Generate temp binary for calc CRC
	COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/link.sh
		${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${PROJECT_NAME}.elf.dir/link.txt
		`crc32 ${PROJECT_NAME}.temp.bin` `stat --printf="%s" ${PROJECT_NAME}.temp.bin`

	BYPRODUCTS ${NAME}.hex
	COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${NAME}.hex

	BYPRODUCTS ${NAME}.bin
	COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${NAME}.bin --gap-fill=0xFF

	COMMENT "Generate hex and bin"
	)

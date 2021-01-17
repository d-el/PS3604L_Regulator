cmake_minimum_required (VERSION 2.6)

project (modbustester)

set(COMMONFLAGS "\
	-g3 -Og \
	")
	
set(CMAKE_CXX_FLAGS "${CPUFLAGS} ${COMMONFLAGS} -std=c++17 -fno-rtti -fno-exceptions")
set(CMAKE_C_FLAGS "${CPUFLAGS} ${COMMONFLAGS} -std=gnu11")

add_subdirectory(libmodbus)
add_subdirectory(parent/utils)
add_subdirectory(parent/prmsystem)

add_executable(modbustester
	modbustester.cpp
	prmsystem/prmsystem.cpp
	)

include_directories(prmsystem)

target_link_libraries(${PROJECT_NAME}
	libmodbus
	utils
	prmsystem
	)

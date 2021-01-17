cmake_minimum_required (VERSION 2.6)

project (prmsytem2json)

set(COMMONFLAGS "\
	-g3 -Og \
	")
	
set(CMAKE_CXX_FLAGS "${CPUFLAGS} ${COMMONFLAGS} -std=c++17")
set(CMAKE_C_FLAGS "${CPUFLAGS} ${COMMONFLAGS} -std=gnu11")

add_executable(prmsytem2json
	prmsytem2json.cpp
	)

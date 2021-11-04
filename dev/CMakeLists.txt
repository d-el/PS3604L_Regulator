add_library(dev STATIC
	src/ds18b20.c
	src/ina229.c
	src/oneWireUart.c
	)

target_include_directories(dev PUBLIC
	include
	)

target_link_libraries(dev drivers utils freertos)

add_library(dev STATIC
	src/ad468x.c
	src/ad5663.c
	src/ds18b20.c
	src/oneWireUart.c
	)

target_include_directories(dev PUBLIC
	include
	)

target_compile_options(dev PRIVATE -DOW_RESETBAUD=9000 -DOW_RWBITBAUD=120000)

target_link_libraries(dev drivers utils freertos)

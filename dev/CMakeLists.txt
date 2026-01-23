add_library(dev STATIC
	src/ad468x.c
	src/ad5060.c
	src/ds18b20.cpp
	)

target_include_directories(dev
	PUBLIC
		include
	PRIVATE
		include/dev
	)

target_compile_options(dev PRIVATE -DOW_RESETBAUD=9000 -DOW_RWBITBAUD=120000)

target_link_libraries(dev hal driver utils freertos)

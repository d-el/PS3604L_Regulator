add_library(utils STATIC
	src/crc.c
	src/plog.c
	)

target_include_directories(utils PUBLIC
	include
	)

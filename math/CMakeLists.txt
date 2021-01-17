add_library(math STATIC
	specificMath.c
	)

target_include_directories(math PUBLIC
	./
	)

target_link_libraries(math iqmathlib-cm4)

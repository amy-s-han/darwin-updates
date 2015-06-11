# CMakeLists.txt for darwin/Linux/project/tutorial/read_write

set(sources
	main.cpp )

add_executable(read_write ${sources})
target_link_libraries(read_write darwin pthread rt)
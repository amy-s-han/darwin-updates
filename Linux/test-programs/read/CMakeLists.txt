# CMakeLists.txt for darwin/Linux/test-programs/test_read

set(sources
	test_read.cpp
	ports.cpp
	)

add_executable(test_read ${sources})

target_link_libraries(test_read darwin pthread rt)


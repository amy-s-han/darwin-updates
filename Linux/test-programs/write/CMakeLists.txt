# CMakeLists.txt for darwin/Linux/test-programs/test_write

set(sources
	test_write.cpp
	ports.cpp
	)

add_executable(test_write ${sources})

target_link_libraries(test_write darwin pthread rt)


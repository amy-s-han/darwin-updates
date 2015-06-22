# CMakeLists.txt for darwin/Linux/test-programs

set(sources
	test.cpp
	ports.cpp
	)

add_executable(test ${sources})

target_link_libraries(test darwin pthread rt)


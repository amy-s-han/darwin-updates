# CMakeLists.txt for darwin/Linux/test-programs/Demo

set(sources
	demo.cpp
	ports.cpp
	)

add_executable(demo_test ${sources})

target_link_libraries(demo_test darwin pthread rt)


# CMakeLists.txt for darwin/Linux/test-programs/test_LED

set(sources
	test_LED.cpp
	ports.cpp
	)

add_executable(test_LED ${sources})

target_link_libraries(test_LED darwin pthread rt)


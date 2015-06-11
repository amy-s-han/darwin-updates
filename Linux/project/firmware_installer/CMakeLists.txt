# CMakeLists.txt for darwin/Linux/project/firmware_installer

set(sources
	hex2bin.cpp
	main.cpp )

add_executable(firmware_installer ${sources})
target_link_libraries(firmware_installer darwin pthread rt)
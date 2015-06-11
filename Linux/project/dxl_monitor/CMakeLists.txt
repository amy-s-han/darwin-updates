# CMakeLists.txt for darwin/Linux/project/dxl_monitor

set(sources
	cmd_process.cpp
	main.cpp )

add_executable(dxl_monitor ${sources})
target_link_libraries(dxl_monitor darwin pthread rt)
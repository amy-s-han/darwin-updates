# CMakeLists.txt for darwin/Linux/project/offset_tuner

set(sources
	cmd_process.cpp
	main.cpp )

add_executable(offset_tuner ${sources})
target_link_libraries(offset_tuner darwin pthread ncurses rt jpeg)
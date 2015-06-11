# CMakeLists.txt for darwin/Linux/project/walk_tuner

set(sources
	cmd_process.cpp
	main.cpp )

add_executable(walk_tuner ${sources})
target_link_libraries(walk_tuner darwin pthread ncurses rt jpeg)
# CMakeLists.txt for darwin/Linux/project/action_editor

set(sources
	cmd_process.cpp
	main.cpp )

add_executable(action_editor ${sources})

target_link_libraries(action_editor darwin pthread ncurses rt)
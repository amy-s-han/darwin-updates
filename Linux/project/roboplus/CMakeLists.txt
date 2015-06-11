# CMakeLists.txt for darwin/Linux/project/roboplus

set(sources
	main.cpp )

add_executable(roboplus ${sources})
target_link_libraries(roboplus darwin pthread ncurses rt)
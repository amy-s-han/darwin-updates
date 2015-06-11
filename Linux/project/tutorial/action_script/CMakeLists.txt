# CMakeLists.txt for darwin/Linux/project/tutorial/action_script

set(sources
	main.cpp )

add_executable(action_script ${sources})
target_link_libraries(action_script darwin pthread jpeg rt)
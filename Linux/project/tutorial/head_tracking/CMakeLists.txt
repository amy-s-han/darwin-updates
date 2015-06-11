# CMakeLists.txt for darwin/Linux/project/tutorial/head_tracking

set(sources
	main.cpp )

add_executable(head_tracking ${sources})
target_link_libraries(head_tracking darwin pthread jpeg rt)
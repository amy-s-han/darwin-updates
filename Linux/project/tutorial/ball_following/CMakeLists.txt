# CMakeLists.txt for darwin/Linux/project/tutorial/ball_following

set(sources
	main.cpp )

add_executable(ball_following ${sources})
target_link_libraries(ball_following darwin pthread jpeg rt)
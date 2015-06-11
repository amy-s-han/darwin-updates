# CMakeLists.txt for darwin/Linux/project/tutorial/camera_tutorial

set(sources
	main.cpp )

add_executable(camera_tutorial ${sources})
target_link_libraries(camera_tutorial darwin pthread jpeg)
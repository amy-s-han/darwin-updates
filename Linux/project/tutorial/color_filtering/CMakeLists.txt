# CMakeLists.txt for darwin/Linux/project/tutorial/color_filtering

set(sources
	main.cpp )

add_executable(color_filtering ${sources})
target_link_libraries(color_filtering darwin pthread jpeg)
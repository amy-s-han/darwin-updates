# CMakeLists.txt for darwin/Linux/project/tutorial/fsr

set(sources
	main.cpp )

add_executable(fsr ${sources})
target_link_libraries(fsr darwin pthread rt jpeg)
# CMakeLists.txt for darwin/Linux/test-programs/redesign

set(sources
	darwinController.cpp
	CM730.cpp
	#playback_redesign.cpp
	test.cpp
	)

add_executable(test ${sources})

target_link_libraries(test darwin pthread rt)


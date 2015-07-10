# CMakeLists.txt for darwin/Linux/test-programs/redesign

set(sources
	darwinController.cpp
	CM730.cpp
	playback_redesign.cpp
	)

add_executable(playback_redesign ${sources})

target_link_libraries(playback_redesign darwin pthread rt)


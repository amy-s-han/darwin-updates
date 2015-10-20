# CMakeLists.txt for darwin/redesign

set(sources
	darwinController.cpp
	CM730.cpp
	playback_redesign.cpp
	#test.cpp
	#pidTuning.cpp
	)

add_executable(playback_redesign ${sources})
#add_executable(test ${sources})
#add_executable(pidTuning ${sources})

target_link_libraries(playback_redesign darwin pthread rt)
#target_link_libraries(test darwin pthread rt)
#target_link_libraries(pidTuning darwin pthread rt)


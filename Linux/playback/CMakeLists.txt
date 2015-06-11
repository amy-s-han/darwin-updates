# CMakeLists.txt for darwin/Linux/playback

set(sources
	playback.cpp
	playback_module.cpp )

add_executable(playback ${sources})

target_link_libraries(playback control_robot ${DARWIN_LIBS} ${SDL_LIBRARIES})


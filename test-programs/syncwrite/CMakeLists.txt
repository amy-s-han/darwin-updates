
# CMakeLists.txt for darwin/test-programs/syncwrite

set(sources
        syncwrite.cpp
	ports.cpp
	)

add_executable(syncwrite ${sources})

target_link_libraries(syncwrite darwin pthread rt)


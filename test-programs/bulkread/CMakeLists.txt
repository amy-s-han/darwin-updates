# CMakeLists.txt for darwin/test-programs/bulkread

set(sources
	bulkread.cpp
	ports.cpp
	)

add_executable(bulkread ${sources})

target_link_libraries(bulkread darwin pthread rt)


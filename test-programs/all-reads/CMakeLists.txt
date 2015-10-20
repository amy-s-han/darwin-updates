# CMakeLists.txt for darwin/test-programs/all-reads

set(sources
	all-reads.cpp
	ports.cpp
	)

add_executable(all-reads ${sources})

target_link_libraries(all-reads darwin pthread rt)


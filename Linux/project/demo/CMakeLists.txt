# CMakeLists.txt for darwin/Linux/project/demo

set(sources
  VisionMode.cpp
  StatusCheck.cpp
  main.cpp)

add_executable(demo ${sources})
target_link_libraries(demo darwin pthread jpeg rt)
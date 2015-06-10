# Top-level CMakeLists.txt for darwin software
cmake_minimum_required(VERSION 2.6)
project(darwin)

include_directories(
  Linux/include
  Framework/include)
  
add_subdirectory(Linux)

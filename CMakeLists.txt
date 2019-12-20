cmake_minimum_required(VERSION 3.5.1)
project(point_cloud_io)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

cs_add_executable(read
  src/read_node.cpp
  src/Read.cpp
)

cs_add_executable(write
  src/write_node.cpp
  src/Write.cpp
)

cs_install()
cs_export()

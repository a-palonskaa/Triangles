cmake_minimum_required(VERSION 3.12)
project(triangles_intersection)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INCLUDE_DIR ./include)
set(SRC_DIR ./src)
set(TEST_DIR ./test)

add_executable(find ${SRC_DIR}/data_structures.cpp ${SRC_DIR}/find_intersection.cpp)
targ
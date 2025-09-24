cmake_minimum_required(VERSION 3.12)
project(triangles_intersection)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INCLUDE_DIR ./include)
set(SRC_DIR ./src)
set(TEST_DIR ./test)

add_executable(find ${SRC_DIR}/data_structures.cpp ${SRC_DIR}/find_intersection.cpp)
target_include_directories(find PRIVATE ${INCLUDE_DIR})

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.tar.gz
)

FetchContent_MakeAvailable(googletest)

add_executable(run_tests ${TEST_DIR}/gtests.cpp)
target_include_directories(run_tests PRIVATE ${INCLUDE_DIR})
target_link_libraries(run_tests PRIVATE GTest::gtest_main)

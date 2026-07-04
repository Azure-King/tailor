cmake_minimum_required(VERSION 3.16)
project(tailor_test CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)



add_executable(test_polygon_data test_polygon_data.cpp)
target_include_directories(test_polygon_data PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

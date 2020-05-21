cmake_minimum_required(VERSION 3.6)

project(DelaunyTriangulation VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS true)
set(CMAKE_CXX_STANDARD 17)

## Eigen3 Support

set(PACKAGE_PREFIX_DIR ${CMAKE_SOURCE_DIR}/libs)
set(Eigen3_DIR ${CMAKE_SOURCE_DIR}/libs/share/eigen3/cmake)

find_package(Eigen3 REQUIRED)
include(${EIGEN3_USE_FILE})

## Demo

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set (CMAKE_CXX_FLAGS_DEBUG "-Wall -Werror ${CMAKE_CXX_FLAGS_DEBUG}")
set (EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin")

file(GLOB_RECURSE TRIANGULATION_SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)
include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(triangulation ${TRIANGULATION_SOURCES})
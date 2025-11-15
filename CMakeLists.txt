cmake_minimum_required(VERSION 3.24)

project(gps_oled CXX)

find_package(Boost REQUIRED)

set(CMAKE_CXX_STANDARD 20)

add_executable(gps_oled)

include(cmake/utilities.cmake)

add_subdirectory(src)


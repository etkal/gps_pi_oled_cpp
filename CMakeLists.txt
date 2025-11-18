cmake_minimum_required(VERSION 3.24)

# Enable WS2812 LED
add_compile_definitions(ENABLE_WS2812)

project(gps_oled CXX)

find_package(Boost REQUIRED)

set(CMAKE_CXX_STANDARD 20)

add_executable(gps_oled)
target_link_libraries(
    gps_oled
    PRIVATE
        ws281x
    PUBLIC
        Boost::headers
)

include(cmake/utilities.cmake)

add_subdirectory(src)


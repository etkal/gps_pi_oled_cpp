cmake_minimum_required(VERSION 3.24)

project(ws281x)

set(CMAKE_C_STANDARD 11)

set(LIB_TARGET ws281x)

set(LIB_PUBLIC_HEADERS
    ws2811.h
    rpihw.h
    pwm.h
    clk.h
    dma.h
    gpio.h
    mailbox.h
    pcm.h
)

set(LIB_SOURCES
    mailbox.c
    ws2811.c
    pwm.c
    pcm.c
    dma.c
    rpihw.c
)

include(GNUInstallDirs)

set(DEST_HEADERS_DIR include/ws2811)
set(DEST_LIB_DIR lib)

add_library(${LIB_TARGET} SHARED ${LIB_SOURCES})

target_link_libraries(${LIB_TARGET} m)
set_target_properties(${LIB_TARGET} PROPERTIES PUBLIC_HEADER "${LIB_PUBLIC_HEADERS}")


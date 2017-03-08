cmake_minimum_required (VERSION 2.6)

set(CMAKE_C_FLAGS "-O2 -Wall")
include_directories(../../lora_gateway/libloragw/inc)

add_definitions(-DENABLE_HAL_UBX)

list(APPEND forwarder_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/forwarder.c
    ${CMAKE_CURRENT_SOURCE_DIR}/parson.c
)

find_library(MATH_LIB m)
#find_library(BLKID_LIB blkid)

add_executable(forwarder ${forwarder_sources})

target_link_libraries(forwarder 
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lora_gateway/libloragw/libloragw.a
    ${MATH_LIB}
)
#    ${BLKID_LIB}
#    ${P_LIB}




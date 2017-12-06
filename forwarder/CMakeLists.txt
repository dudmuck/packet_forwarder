cmake_minimum_required (VERSION 2.6)

set(CMAKE_C_FLAGS "-O2 -Wall")
include_directories(../../lora_gateway/libloragw/inc)

add_definitions(-DENABLE_HAL_UBX)

find_library(MATH_LIB m)
find_library(P_LIB pthread)

list(APPEND forwarder_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/parson.c
)

if (16ch)
    list(APPEND forwarder_sources
        ${CMAKE_CURRENT_SOURCE_DIR}/forwarder_16ch.c
    )
    add_executable(forwarder_16ch ${forwarder_sources})
    target_link_libraries(forwarder_16ch
        ${CMAKE_CURRENT_SOURCE_DIR}/../../lora_gateway/libloragw/libloragw.a
        ${MATH_LIB}
        ${P_LIB}
    )
else (16ch)
    list(APPEND forwarder_sources
        ${CMAKE_CURRENT_SOURCE_DIR}/forwarder.c
    )
    add_executable(forwarder ${forwarder_sources})
    target_link_libraries(forwarder
        ${CMAKE_CURRENT_SOURCE_DIR}/../../lora_gateway/libloragw/libloragw.a
        ${MATH_LIB}
        ${P_LIB}
    )
endif (16ch)


list(APPEND multi_fwd_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/multi_fwd.c
)

add_executable(multi_fwd ${multi_fwd_sources})
target_link_libraries(multi_fwd
    json-c
    ${MATH_LIB}
    ${P_LIB}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lora_gateway/libloragw/libloragw.a
)

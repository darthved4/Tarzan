cmake_minimum_required(VERSION 3.20.0)

set(DTC_OVERLAY_FILE "boards/app.overlay" CACHE STRING
    "Device-tree overlay used by the selected Tarzan application")
set(TARZAN_APP_SOURCE "src/main.c" CACHE STRING
    "Tarzan application entry point")

if(NOT IS_ABSOLUTE "${DTC_OVERLAY_FILE}")
    set(DTC_OVERLAY_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${DTC_OVERLAY_FILE}"
        CACHE STRING "Device-tree overlay used by the selected Tarzan application"
        FORCE)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(Tarzan LANGUAGES C)

target_sources(app PRIVATE ${TARZAN_APP_SOURCE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wpedantic -Wall -Wextra")

execute_process(
    COMMAND git -C ${CMAKE_CURRENT_SOURCE_DIR} describe --always --broken --dirty --tags
    OUTPUT_VARIABLE TARZAN_GIT_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
add_definitions(-DTARZAN_GIT_VERSION=\"${TARZAN_GIT_VERSION}\")

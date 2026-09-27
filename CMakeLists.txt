cmake_minimum_required(VERSION 3.22)
project(teseo)

set(CMAKE_CXX_STANDARD 20)

# Default to an optimized build. Without this, single-config generators
# (Makefiles, Ninja) build with no optimization at all, which makes any
# performance measurement meaningless. Override with -DCMAKE_BUILD_TYPE=Debug
# while debugging, or use the CMake Tools variant selector in Visual Studio Code.
get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(NOT IS_MULTI_CONFIG AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

include(FetchContent)

FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/6.0.tar.gz
)

FetchContent_Declare(
    box2d
    URL https://github.com/erincatto/box2d/archive/refs/tags/v3.1.1.tar.gz
)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(raylib box2d)

set(SOURCES
    src/main.cpp
    src/sim/agent.cpp
    src/sim/maze.cpp
    src/sim/sim.cpp
    src/ui/ui.cpp
    src/starter_mouse/starter_mouse.cpp
    src/keyboard_mouse/keyboard_mouse.cpp
)

add_executable(teseo ${SOURCES})

target_link_libraries(teseo PRIVATE raylib box2d::box2d)

target_include_directories(teseo PRIVATE src)

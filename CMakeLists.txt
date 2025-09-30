cmake_minimum_required(VERSION 3.16)
project(3DG-pbr)

set(CMAKE_CXX_COMPILER "/usr/bin/clang++" CACHE FILEPATH "C++ compiler" FORCE)
set(CMAKE_C_COMPILER "/usr/bin/clang" CACHE FILEPATH "C compiler" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add Externals
#...

# Include folders
include_directories(include)

# Add executable early
add_executable(render "tests/main.cpp")

# Use CMake's built-in OpenMP support
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "Found OpenMP: ${OpenMP_CXX_INCLUDE_DIRS}")
    target_link_libraries(vol_render PRIVATE OpenMP::OpenMP_CXX)
endif()

cmake_minimum_required(VERSION 3.16)
project(3DG-rt)

set(CMAKE_CXX_COMPILER "/usr/bin/clang++" CACHE FILEPATH "C++ compiler" FORCE)
set(CMAKE_C_COMPILER "/usr/bin/clang" CACHE FILEPATH "C compiler" FORCE)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add Externals
include_directories("extern/eigen")
include_directories("extern/glfw")
include_directories("extern/argparse/include")

# Include folders
include_directories(include)

# Add executable early
file(GLOB SRC_FILES "src/*.cpp")

add_executable(render ${SRC_FILES})

# Use CMake's built-in OpenMP support
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "Found OpenMP: ${OpenMP_CXX_INCLUDE_DIRS}")
    target_link_libraries(render PRIVATE OpenMP::OpenMP_CXX)
endif()

# Add GLFW and OpenGL linking
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

target_link_libraries(render PRIVATE glfw OpenGL::GL)
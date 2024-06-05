cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(mad-icp LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS -Werror)

# this is required only to compile apps in C++, if you work in Python this can be set to off
# if need to compile C++ apps, when executing CMake run `cmake -DCOMPILE_CPP_APPS=ON ..`
option(COMPILE_CPP_APPS "Set to ON to compile C++ applications" OFF)

# Eigen 
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# OpenMP 
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

# Python bindings
find_package(pybind11 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
)

add_subdirectory(src)
# TODO only if one wants to build cpp runners
add_subdirectory(apps/cpp_runners)
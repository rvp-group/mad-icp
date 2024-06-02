cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(mad-icp LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS -Werror)

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


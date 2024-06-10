# TODO: Change version to at least 3.11 and include FetchContent for Eigen
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(mad_icp LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS -Werror)

# this is required only to compile apps in C++, if you work in Python this can be set to off
# if need to compile C++ apps, when executing CMake run `cmake -DCOMPILE_CPP_APPS=ON ..`
option(COMPILE_CPP_APPS "Set to ON to compile C++ applications" OFF)

# Eigen
# TODO: Detect if Eigen exists in system, oterwhise, download it through FetchContent
find_package(Eigen3 3.3 NO_MODULE)

if(not Eigen3_FOUND)
    message(CHECK_START "Fetching Eigen3")
    list(APPEND CMAKE_MESSAGE_INDENT "  ")

    include(FetchContent)
    FetchContent_Declare(
        Eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE)

    # note: To disable eigen tests,
    # you should put this code in a add_subdirectory to avoid to change
    # BUILD_TESTING for your own project too since variables are directory
    # scoped
    set(BUILD_TESTING OFF)
    set(EIGEN_BUILD_TESTING OFF)
    set(EIGEN_MPL2_ONLY ON)
    set(EIGEN_BUILD_PKGCONFIG OFF)
    set(EIGEN_BUILD_DOC OFF)
    FetchContent_MakeAvailable(Eigen)

    list(POP_BACK CMAKE_MESSAGE_INDENT)
    message(CHECK_PASS "fetched")
endif()

# OpenMP
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
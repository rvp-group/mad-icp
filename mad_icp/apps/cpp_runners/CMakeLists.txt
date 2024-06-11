if(COMPILE_CPP_APPS)
  message("Compiling CPP Apps")

  find_package(yaml-cpp)

  if(NOT DEFINED yaml-cpp_FOUND)
    message(CHECK_START "Fetching pybind11")
    list(APPEND CMAKE_MESSAGE_INDENT "  ")
    include(FetchContent)
    FetchContent_Declare(
      yaml-cpp
      GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
      GIT_TAG 0.8.0 # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
    )
    FetchContent_GetProperties(yaml-cpp)

    if(NOT yaml-cpp_POPULATED)
      message(STATUS "Fetching yaml-cpp...")
      FetchContent_Populate(yaml-cpp)
      add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
    endif()

    message(CHECK_PASS "fetched")
  endif()

  add_executable(bin_runner bin_runner.cpp)
  target_link_libraries(bin_runner
    odometry
    yaml-cpp
  )
endif()
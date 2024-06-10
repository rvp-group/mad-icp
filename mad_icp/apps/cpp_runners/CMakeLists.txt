if(COMPILE_CPP_APPS)
  message("Compiling CPP Apps")
  add_executable(bin_runner bin_runner.cpp)
  target_link_libraries(bin_runner
    odometry
    yaml-cpp
  )
endif()
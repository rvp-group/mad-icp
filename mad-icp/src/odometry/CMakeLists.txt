add_library(odometry SHARED
  vel_estimator.cpp
  pipeline.cpp
  mad_icp.cpp
)

target_link_libraries(odometry
  tools
)

target_compile_features(odometry PUBLIC)

pybind11_add_module(pypeline pybind/pypeline.cpp)
target_link_libraries(pypeline PUBLIC
  odometry
  tools
)


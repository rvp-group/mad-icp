add_library(odometry STATIC
  vel_estimator.cpp
  pipeline.cpp
  mad_icp.cpp
)

target_link_libraries(odometry
  tools
)

target_compile_features(odometry PUBLIC)
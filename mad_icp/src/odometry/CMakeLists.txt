add_library(odometry STATIC
  vel_estimator.cpp
  pipeline.cpp
  mad_icp.cpp
)

target_link_libraries(odometry
  tools
  Eigen3::Eigen
)
target_compile_features(odometry PUBLIC)
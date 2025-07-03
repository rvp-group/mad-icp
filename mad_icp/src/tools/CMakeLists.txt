add_library(tools STATIC
  mad_tree.cpp
)

ament_target_dependencies(tools
  sensor_msgs
)

target_link_libraries(tools Eigen3::Eigen)
target_compile_features(tools PUBLIC)
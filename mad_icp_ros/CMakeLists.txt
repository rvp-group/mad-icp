cmake_minimum_required(VERSION 3.8)
project(mad_icp_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 3.3 NO_MODULE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(mad_icp REQUIRED)


# add a library
add_library(mad_icp_ros_components SHARED 
  src/odometry.cpp
)

# dependencies
ament_target_dependencies(mad_icp_ros_components 
  rclcpp 
  rclcpp_components
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

target_link_libraries(mad_icp_ros_components
  odometry
  yaml-cpp
)

# OpenMP (I should not need to do this)
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
#


# headers
target_include_directories(mad_icp_ros_components PUBLIC
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
	"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

rclcpp_components_register_node(
  mad_icp_ros_components
  PLUGIN "mad_icp_ros::Odometry"
  EXECUTABLE mad_icp_odometry
)

# install rules
install(
  TARGETS mad_icp_ros_components
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()

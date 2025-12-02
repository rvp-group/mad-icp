cmake_minimum_required(VERSION 3.8)

project(mad_icp_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

find_package(Eigen3 3.3 NO_MODULE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(mad_icp REQUIRED)
find_package(mad_icp_ros_interfaces REQUIRED)

find_package(visualization_msgs REQUIRED)

# add a library
add_library(mad_icp_ros_components SHARED 
  src/map_server.cpp
  src/localization.cpp
  src/odometry.cpp
  src/odom_cbuf.cpp
)

# dependencies
ament_target_dependencies(mad_icp_ros_components 
  rclcpp 
  rclcpp_components
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  mad_icp_ros_interfaces
  pcl_conversions
  pcl_ros
  visualization_msgs
)

target_link_libraries(mad_icp_ros_components
  odometry
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

rclcpp_components_register_node(
  mad_icp_ros_components
  PLUGIN "mad_icp_ros::Localizer"
  EXECUTABLE mad_icp_localizer
)

rclcpp_components_register_node(
  mad_icp_ros_components
  PLUGIN "mad_icp_ros::MapServer"
  EXECUTABLE mad_icp_map_server
)

# install rules
install(
  TARGETS mad_icp_ros_components
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# launch files
install( DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch      
		 FILES_MATCHING PATTERN "*.launch.py" 
)

ament_package()

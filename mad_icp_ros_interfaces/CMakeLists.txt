cmake_minimum_required(VERSION 3.8)
project(mad_icp_ros_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MadNode.msg" "msg/MadTree.msg" "msg/Frame.msg" 
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

find_package(mad_icp REQUIRED)
find_package(Eigen3 3.3 NO_MODULE)

# add a library
add_library(mad_icp_ros_utils SHARED 
  src/utils.cpp
)

# dependencies
ament_target_dependencies(mad_icp_ros_utils 
  rclcpp 
  sensor_msgs
)

target_link_libraries(mad_icp_ros_utils
  tools
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(mad_icp_ros_utils "${cpp_typesupport_target}")

# headers
target_include_directories(mad_icp_ros_utils PUBLIC
	"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
	"$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")


# install rules
install(
  TARGETS mad_icp_ros_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install message files
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(
  include
)

ament_package()

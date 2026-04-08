set(BUILD_TESTS OFF)
set(BUILD_EXAMPLES OFF)

add_executable(odom_node
  src/odom_node.cpp
)

target_include_directories(odom_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(odom_node 
  std_msgs::std_msgs__rosidl_typesupport_cpp
  # Python3::Python
)

target_compile_features(odom_node PUBLIC c_std_99 cxx_std_17)
target_compile_options(odom_node PRIVATE -Wno-psabi)

ament_target_dependencies(odom_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS odom_node
  DESTINATION lib/${PROJECT_NAME}
)

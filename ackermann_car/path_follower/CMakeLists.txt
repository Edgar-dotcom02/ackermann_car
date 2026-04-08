set(BUILD_TESTS OFF)
set(BUILD_EXAMPLES OFF)

find_package(car_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(path_follower 
  src/path_follower.cpp
  src/pure_pursuit.cpp
)

target_include_directories(path_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(path_follower 
    std_msgs::std_msgs__rosidl_typesupport_cpp
    # Python3::Python
)

target_compile_features(path_follower PUBLIC c_std_99 cxx_std_17)
target_compile_options(path_follower PRIVATE -Wno-psabi)

ament_target_dependencies(path_follower
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  car_interfaces
  visualization_msgs
)

install(TARGETS path_follower
  DESTINATION lib/${PROJECT_NAME}
)
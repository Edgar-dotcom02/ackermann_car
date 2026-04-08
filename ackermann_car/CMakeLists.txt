cmake_minimum_required(VERSION 3.8)
project(ackermann_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(car_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

install(
  DIRECTORY launch model config
  DESTINATION share/${PROJECT_NAME}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-psabi)
endif()

add_subdirectory(vehicle_controller)
add_subdirectory(odom_node)
add_subdirectory(path_follower)

ament_package()

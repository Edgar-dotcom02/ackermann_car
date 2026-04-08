set(BUILD_TESTS OFF)
set(BUILD_EXAMPLES OFF)

add_executable(vehicle_controller 
  src/vehicle_controller.cpp
)

target_include_directories(vehicle_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(vehicle_controller 
  std_msgs::std_msgs__rosidl_typesupport_cpp
)

target_compile_features(vehicle_controller PUBLIC c_std_99 cxx_std_17)
target_compile_options(vehicle_controller PRIVATE -Wno-psabi)

ament_target_dependencies(vehicle_controller 
  rclcpp 
  std_msgs
)

install(TARGETS vehicle_controller
  DESTINATION lib/${PROJECT_NAME}
)
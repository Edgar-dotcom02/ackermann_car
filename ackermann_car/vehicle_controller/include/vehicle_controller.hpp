#ifndef VEHICLE_CONTROLLER_HPP
#define VEHICLE_CONTROLLER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class VehicleController : public rclcpp::Node
{

public:
  VehicleController(const double timer_period = 1e-2, const double timeout_duration = 1e9);

private:

  // Calculates the steering angles for the left and right wheels using the 
  // Ackermann steering geometry.
  std::pair<double, double> ackermann_steering_angle();

  // Calculates the rear differential wheel (left & right) velocities for a vehicle with a rear 
  // differential axle
  std::pair<double, double> rear_differential_velocity();

  void timer_callback();

  void steering_angle_callback(const std_msgs::msg::Float64::SharedPtr msg);

  void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg);

  double timeout_duration_;
  rclcpp::Time last_velocity_time_;
  rclcpp::Time last_steering_time_;

  double body_width_;
  double body_length_;
  double wheel_radius_;
  double wheel_width_;
  double max_steering_angle_;
  double max_velocity_;
  double wheel_base_;
  double track_width_;

  double steering_angle_;
  double velocity_;

  std::vector<double> wheel_angular_velocity_;
  std::vector<double> wheel_steering_angle_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_angle_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // VEHICLE_CONTROLLER_HPP
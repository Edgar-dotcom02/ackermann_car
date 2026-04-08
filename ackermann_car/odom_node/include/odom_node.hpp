// odom_node.hpp
#pragma once

#include <cstdlib>
#include <math.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_broadcaster.h"

class OdomNode : public rclcpp::Node
{
public:
    OdomNode();
    // ~OdomNode();

private:
    void calcOdometry();
    void onCmdTime();

    double getBikeSteering(std::pair<double, double> ackermann_steer_angles);
    double getBikeSpeed(std::pair<double, double> diff_rear_speeds);

    void calcInvKinematics(double target_vx, double target_vtheta);

    void states_cb(const sensor_msgs::msg::JointState::SharedPtr msg);
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void deviceControl_cb(const geometry_msgs::msg::Point::SharedPtr msg);

    // Feedback/Command speed and steering angle
    double m_speed = 0.0; // mps
    double m_steer = 0.0; // rad
    double m_cmdSpeed = 0.0; // mps
    double m_cmdSteer = 0.0; // rad

    // Cmd_vel targets
    double m_targetVx = 0.0; // mps
    double m_targetVtheta = 0.0; // radps

    // State variables
    double m_x = 0.0, m_y = 0.0, m_theta = 0.0; // m and rad
    double m_vx = 0.0, m_vy = 0.0, m_vtheta = 0.0; // mps and radps

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pubCmdSpeed;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pubCmdSteeringPose;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_pubOdometry;
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_subJointStates;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subCmdVel;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr m_subDeviceControl;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;

    bool m_cmdTimedOut = false;

    rclcpp::Time m_lastOdomTime;
    rclcpp::Time m_lastCmdTime;

    rclcpp::TimerBase::SharedPtr m_timeOdom;
    rclcpp::TimerBase::SharedPtr m_timeCmd;

    rclcpp::CallbackGroup::SharedPtr m_cmdCbGroup;

    // Constant params
    double WHEELBASE; // m
    double WHEEL_R; // m
    double MAX_SPEED; // m/s
    double STEER_POSE_MAX; // abs of max steering angle (rad)
    double CMD_TIMEOUT;
    double CAR_WIDTH;
    double CAR_LENGTH;
    double TRACK_WIDTH;
    double WHEEL_WIDTH;
    std::string ODOM_FRAME;
    std::string BASE_FRAME;

    // Helper functions
    // inline double rad2deg (double rad) { return static_cast<double>(rad * 180.0 / M_PI); }
    // inline double mps2radps(double mps) { return static_cast<double>(mps / WHEEL_R); }
    // inline double radps2mps(double radps) { return static_cast<double>(radps * WHEEL_R); }
    // inline double deg2rad (double deg) { return static_cast<double>(deg * M_PI / 180.0 ); }
};

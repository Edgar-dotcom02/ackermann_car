// odom_node.cpp
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "odom_node.hpp"

using namespace std::chrono_literals; 

OdomNode::OdomNode() : Node("odom_node")
{
    RCLCPP_INFO(this->get_logger(), "Starting odom_node");

    // Loading constant params
    this->declare_parameter<double>("body_width", 0.0);
    this->declare_parameter<double>("body_length", 0.0);
    this->declare_parameter<double>("wheel_radius", 0.0);
    this->declare_parameter<double>("max_steering_angle", 0.0);
    this->declare_parameter<double>("max_velocity", 0.0);
    this->declare_parameter<double>("cmd_timeout", 0.0);
    this->declare_parameter<double>("wheel_width", 0.0);
    this->declare_parameter<std::string>("odom_frame", "");
    this->declare_parameter<std::string>("base_frame", "");
    
    this->get_parameter("body_width", CAR_WIDTH);
    this->get_parameter("body_length", CAR_LENGTH);
    this->get_parameter("wheel_radius", WHEEL_R);
    this->get_parameter("max_steering_angle", STEER_POSE_MAX);
    this->get_parameter("max_velocity", MAX_SPEED);
    this->get_parameter("wheel_width", WHEEL_WIDTH);
    this->get_parameter("cmd_timeout", CMD_TIMEOUT);
    this->get_parameter("odom_frame", ODOM_FRAME);
    this->get_parameter("base_frame", BASE_FRAME); 

    WHEELBASE = CAR_LENGTH - (2 * WHEEL_R);
    TRACK_WIDTH = CAR_WIDTH + (2 * WHEEL_WIDTH / 2.0);

    // Callback group
    m_cmdCbGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions cmd_sub_options;
    cmd_sub_options.callback_group = m_cmdCbGroup;

    // Publishers and subscribers
    m_pubCmdSpeed = this->create_publisher<std_msgs::msg::Float64>(
        "/velocity", 10);
    m_pubCmdSteeringPose = this->create_publisher<std_msgs::msg::Float64>(
        "/steering_angle", 10);

    m_pubOdometry = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10);

    m_subJointStates = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&OdomNode::states_cb, this, std::placeholders::_1)
    );
    m_subCmdVel = this->create_subscription<geometry_msgs::msg::Twist>(
       "/cmd_vel", 10,
       std::bind(&OdomNode::cmd_vel_cb, this, std::placeholders::_1),
       cmd_sub_options
    );
    m_subDeviceControl = this->create_subscription<geometry_msgs::msg::Point>(
        "/device_control", 10,
        std::bind(&OdomNode::deviceControl_cb, this, std::placeholders::_1),
        cmd_sub_options
    );

    m_tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    m_timeOdom = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&OdomNode::calcOdometry, this)
    );
    m_timeCmd = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&OdomNode::onCmdTime, this)
    );

    m_lastOdomTime = this->get_clock()->now();
    m_lastCmdTime = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Odom node started");
}

void OdomNode::states_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    double fl_steer = 0.0;
    double fr_steer = 0.0;
    double rl_vel = 0.0;
    double rr_vel = 0.0;

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
        const std::string& name = msg->name[i];

        if (name == "front_left_steering_joint")
        {
            fl_steer = msg->position[i];
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            //     "Front left joint: %.2f", fl_steer);
        }
        else if (name == "front_right_steering_joint")
        {
            fr_steer = msg->position[i];
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            //     "Front right joint: %.2f", fr_steer);
        }
        else if (name == "rear_left_wheel_joint")
        {
            rl_vel = msg->velocity[i];
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            //     "Rear left vel: %.2f", rl_vel);
        }
        else if (name == "rear_right_wheel_joint")
        {
            rr_vel = msg->velocity[i];
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            //     "Rear right vel: %.2f", rr_vel);
        }
    }

    m_speed = getBikeSpeed({rl_vel, rr_vel});
    m_steer = getBikeSteering({fl_steer, fr_steer});
}

double OdomNode::getBikeSteering(std::pair<double, double> ackermann_steer_angles)
{
    const double left_angle = ackermann_steer_angles.first;
    const double right_angle = ackermann_steer_angles.second;

    if (fabs(left_angle) < 1e-4 && fabs(right_angle) < 1e-4)
        return 0.0;

    double left_steer  = std::atan(WHEELBASE * std::tan(left_angle) /
                              (WHEELBASE + (TRACK_WIDTH / 2.0) * std::tan(left_angle)));

    double right_steer = std::atan(WHEELBASE * std::tan(right_angle) /
                              (WHEELBASE - (TRACK_WIDTH / 2.0) * std::tan(right_angle)));

    double steer_angle = 0.5 * (left_steer + right_steer);

    if (fabs(steer_angle) < 1e-3) steer_angle = 0.0;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, 
    //         "Current steering angle: %.2f", steer_angle);
    
    return steer_angle;
}

double OdomNode::getBikeSpeed(std::pair<double, double> diff_rear_speeds)
{
    const double left_wheel_speed = diff_rear_speeds.first;
    const double right_wheel_speed = diff_rear_speeds.second;

    double wheel_speed = 0.5 * (left_wheel_speed + right_wheel_speed); 
    double bike_speed = wheel_speed * WHEEL_R;
    if (fabs(bike_speed) < 1e-3) bike_speed = 0.0;

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500, 
    //         "Current velocity: %.2f", bike_speed);

    return (bike_speed);
}

void OdomNode::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    bool valid = true;

    valid = valid && (!std::isnan(msg->linear.x));
    valid = valid && (!std::isnan(msg->angular.z));

    if (valid) 
    {
        m_targetVtheta = msg->angular.z;
        m_targetVx = msg->linear.x;
        calcInvKinematics(m_targetVx, m_targetVtheta);
        m_lastCmdTime = this->get_clock()->now();
    }
    else
    {
        m_targetVx = 0.0f;
        m_targetVtheta = 0.0f;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Received invalid twist command");
    }
}

// Takes the target speed (m/s) and steering angle (deg) as a point vector
// Where point.x - target_speed and point.z - target steering angle
void OdomNode::deviceControl_cb(const geometry_msgs::msg::Point::SharedPtr msg)
{
    if (!std::isnan(msg->x))
    {
        m_cmdSpeed = msg->x;
        m_lastCmdTime = this->get_clock()->now();
    }
    else
    {
        // m_cmdSpeed = 0.0f;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Received invalid target linear speed!");
    }

    if (!std::isnan(msg->z))
    {
        m_cmdSteer = msg->z;
        m_lastCmdTime = this->get_clock()->now();
    }
    else
    {
        // m_cmdSteer = 0.0f;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
            "Received invalid target steering pose!");
    }
}

void OdomNode::calcOdometry()
{
    rclcpp::Time current_time = this->get_clock()->now();
    double dt_sec = (current_time - m_lastOdomTime).seconds();

    // Bicycle model kinematics
    double v = m_speed;
    double steer_angle = m_steer;

    // Angular velocity
    m_vtheta = (v / WHEELBASE) * std::tan(steer_angle);

    m_theta += m_vtheta * dt_sec;
    m_theta = atan2(sin(m_theta), cos(m_theta)); // [-pi, pi]

    m_vx = v * cos(m_theta);
    m_vy = v * sin(m_theta);
    m_x += m_vx * dt_sec;
    m_y += m_vy * dt_sec;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, m_theta);

    // Publish transform
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = ODOM_FRAME;
    odom_trans.child_frame_id = BASE_FRAME;
    odom_trans.transform.translation.x = m_x;
    odom_trans.transform.translation.y = m_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();
    m_tfBroadcaster->sendTransform(odom_trans);

    // Publish odometry message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = ODOM_FRAME;
    odom.child_frame_id = BASE_FRAME;
    odom.pose.pose.position.x = m_x;
    odom.pose.pose.position.y = m_y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.pose.covariance[35] = 0.1;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = m_vtheta;
    odom.twist.covariance[0]  = 0.01;
    odom.twist.covariance[7]  = 0.01;
    odom.twist.covariance[35] = 0.01;

    m_pubOdometry->publish(odom);
    m_lastOdomTime = current_time;
}

void OdomNode::calcInvKinematics(double target_vx, double target_vtheta)
{
    const double EPS_V = 1e-3f;

    double v = target_vx;
    double w = target_vtheta;

    // No motion if linear speed is too small
    if (fabs(v) < EPS_V) 
    {
        m_cmdSpeed = 0.0f;
        m_cmdSteer = 0.0f;
        return;
    }

    // Physical yaw rate limit
    double w_max = fabs(v) * tan(STEER_POSE_MAX) / WHEELBASE;
    w = std::clamp(w, -w_max, w_max);

    double steer_angle = atan(WHEELBASE * w / v);

    m_cmdSpeed = v;
    m_cmdSteer = steer_angle;
}

void OdomNode::onCmdTime()
{
    rclcpp::Time now = this->get_clock()->now();

    if ((now - m_lastCmdTime).seconds() > CMD_TIMEOUT)
    {
        if (!m_cmdTimedOut)
        {
            RCLCPP_WARN(this->get_logger(), "cmd timeout — stopping robot");
            m_cmdTimedOut = true;
        }
        m_cmdSpeed = 0.0f;
        m_cmdSteer = 0.0f;
    }
    else
    {
        m_cmdTimedOut = false;
    }
    
    // Publish command messages (target speed and steering pose)
    auto cmd_speed_msg = std_msgs::msg::Float64();
    auto cmd_steer_pose_msg = std_msgs::msg::Float64();

    cmd_speed_msg.data = std::clamp(m_cmdSpeed, -MAX_SPEED, MAX_SPEED);
    cmd_steer_pose_msg.data = std::clamp(m_cmdSteer, 
        -STEER_POSE_MAX, STEER_POSE_MAX);
    
    m_pubCmdSpeed->publish(cmd_speed_msg);
    m_pubCmdSteeringPose->publish(cmd_steer_pose_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<OdomNode>();
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("odom_node"), "%s", e.what());
    }

    rclcpp::shutdown();

    std::cout << "[INFO] Shutting down odom_node." << std::endl;

    return 0;
}
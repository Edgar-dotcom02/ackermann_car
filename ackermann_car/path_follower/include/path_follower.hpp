// path_follower.hpp
#pragma once

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "pure_pursuit.hpp"

#include "car_interfaces/action/follow_path.hpp"

class PathFollower : public rclcpp::Node
{
public:
    using FollowPath = car_interfaces::action::FollowPath;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowPath>;

    explicit PathFollower(const rclcpp::NodeOptions& options);
    ~PathFollower();

private:
    // Action server cbs and variables
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&,
                                    std::shared_ptr<const FollowPath::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle>);
    void handle_accepted(std::shared_ptr<GoalHandle> gh);

    rclcpp_action::Server<FollowPath>::SharedPtr m_actionServer;
    std::shared_ptr<GoalHandle> m_activeGoal{nullptr};

    // Odom feeback and control action callback
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_cb();

    std::vector<pp::Point> loadCSV(const std::string& path);
    void publishMarkers();
    void recordPose();
    void saveTrajectory();
    void abortMission(std::shared_ptr<GoalHandle>& gh, const std::string& reason);

    // Pure Pursuit controller
    std::unique_ptr<pp::PPControl> m_controller;
    std::vector<pp::Point> m_path;
    pp::CarPose m_pose{};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odomSub;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_pubControl; // speed and steer pub
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_pubMarkers; // for rviz
    
    rclcpp::TimerBase::SharedPtr m_controlTimer;

    std::vector<pp::CarPose> m_trajectory; // trajectory recorder

    // Params
    double m_goalTolerance;
    double m_missionTimeout;
    std::string m_csvPath;
    std::string m_trajSavePath;

    // State
    bool m_active = false;
    rclcpp::Time m_missionStart;
};
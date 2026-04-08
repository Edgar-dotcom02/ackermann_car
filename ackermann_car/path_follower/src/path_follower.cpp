// path_follower.cpp
#include <chrono>
#include <cmath>
#include <functional>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <string>

#include "path_follower.hpp"

using namespace std::chrono_literals;

PathFollower::PathFollower(const rclcpp::NodeOptions& options)
: Node("path_follower", options)
{
    // Declaring and loading parameters
    this->declare_parameter<std::string>("csv_path", "");
    this->declare_parameter<std::string>("traj_save_path", "");
    this->declare_parameter<double>("target_speed", 0.0);
    this->declare_parameter<double>("wheelbase", 0.0);
    this->declare_parameter<double>("max_steer", 0.0);
    this->declare_parameter<double>("lookahead_coef", 0.0);
    this->declare_parameter<double>("control_hz", 0.0);
    this->declare_parameter<double>("goal_tolerance", 0.0);
    this->declare_parameter<double>("mission_timeout", 0.0);

    this->declare_parameter<std::string>("ctrl_mode", "pure_pursuit");
    this->declare_parameter<double>("kp", 0.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);

    this->get_parameter("csv_path", m_csvPath);
    this->get_parameter("traj_save_path", m_trajSavePath);
    this->get_parameter("goal_tolerance", m_goalTolerance);
    this->get_parameter("mission_timeout", m_missionTimeout);

    double v, L, max_steer, k, hz, dt;

    this->get_parameter("target_speed", v);
    this->get_parameter("wheelbase", L);
    this->get_parameter("max_steer", max_steer);
    this->get_parameter("lookahead_coef", k);
    this->get_parameter("control_hz", hz);

    dt = 1.0 / hz;

    // Load path
    m_path = loadCSV(m_csvPath);
    RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s",
        m_path.size(), m_csvPath.c_str());

    // Building controller
    m_controller = std::make_unique<pp::PPControl>(v, L, max_steer, k, dt, m_path);

    std::string ctrl_mode;
    double kp = 0.0, ki = 0.0, kd = 0.0; 

    this->get_parameter("ctrl_mode", ctrl_mode);
    this->get_parameter("kp", kp);
    this->get_parameter("ki", ki);
    this->get_parameter("kd", kd);

    // Setting controller mode (pid or pure_pursuit)
    m_controller->setMode(ctrl_mode, kp, ki, kd);

    // Pubs and subs
    m_odomSub = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, 
        std::bind(&PathFollower::odom_cb, this, std::placeholders::_1));

    m_pubControl = create_publisher<geometry_msgs::msg::Point>("/device_control", 10);
    m_pubMarkers = create_publisher<visualization_msgs::msg::MarkerArray>("/path_markers", 
        rclcpp::QoS(1).transient_local());

    // Action server
    m_actionServer = rclcpp_action::create_server<FollowPath>(this, "/follow_path",
        std::bind(&PathFollower::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PathFollower::handle_cancel, this, std::placeholders::_1),
        std::bind(&PathFollower::handle_accepted, this, std::placeholders::_1));

    // Control timer
    const auto period = std::chrono::duration<double>(dt);
    m_controlTimer = create_wall_timer(period,
        std::bind(&PathFollower::control_cb, this));

    // publishMarkers(); // RVIZ markers
    RCLCPP_INFO(this->get_logger(), "PathFollower node started");
}

PathFollower::~PathFollower()
{
    if (!m_trajectory.empty())
    {
        saveTrajectory();
    }
}

void PathFollower::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    m_pose.x = msg->pose.pose.position.x;
    m_pose.y = msg->pose.pose.position.y;

    // Get yaw from orientation quaternion
    const auto& q = msg->pose.pose.orientation;
    m_pose.theta = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                              1.0 - 2.0*(q.y*q.y + q.z*q.z));
}

// Action server fns

rclcpp_action::GoalResponse PathFollower::handle_goal(const rclcpp_action::GoalUUID&,
    std::shared_ptr<const FollowPath::Goal>)
{
    if (m_active)
    {
        RCLCPP_WARN(this->get_logger(), "Request rejected: Mission already running.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PathFollower::handle_cancel(std::shared_ptr<GoalHandle>)
{
    RCLCPP_INFO(this->get_logger(), "Mission cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PathFollower::handle_accepted(std::shared_ptr<GoalHandle> gh)
{
    m_activeGoal = gh;
    m_active = true;
    m_missionStart = this->get_clock()->now();

    m_trajectory.clear();
    RCLCPP_INFO(this->get_logger(), "Mission started");
}

// Control callback
void PathFollower::control_cb()
{
    if (!m_active || !m_activeGoal) return;

    // Timeout
    if ((this->get_clock()->now() - m_missionStart).seconds() > m_missionTimeout)
    {
        abortMission(m_activeGoal, "Mission timeout");
        return;
    }

    // Cancel requested
    if (m_activeGoal->is_canceling())
    {
        abortMission(m_activeGoal, "Mission cancelled");
        return;
    }

    auto control_msg = geometry_msgs::msg::Point();
    control_msg.y = 0.0;

    m_controller->dropPassedWaypoints(m_pose);

    // Goal reached
    const auto& goal = m_controller->goalPoint();

    const double dist_to_goal = std::hypot(m_pose.x - goal.x, m_pose.y - goal.y);

    if (m_controller->pathEmpty() || dist_to_goal < m_goalTolerance)
    {
        // Stop the robot
        control_msg.x = 0.0; control_msg.z = 0.0;
        m_pubControl->publish(control_msg);

        auto result = std::make_shared<FollowPath::Result>();
        result->success = true;
        result->message = "Goal reached";
        m_activeGoal->succeed(result);

        m_active = false;
        saveTrajectory();

        RCLCPP_INFO(this->get_logger(), "Goal reached (dist=%.3f m)", dist_to_goal);
        return;
    }

    // Update controller output and publish
    m_controller->update(m_pose);
    recordPose();

    control_msg.x = m_controller->getCmdSpeed();
    control_msg.z = m_controller->getCmdSteer();
    m_pubControl->publish(control_msg);

    // Feedback
    auto feedback = std::make_shared<FollowPath::Feedback>();
    feedback->distance_to_goal = static_cast<float>(dist_to_goal);
    m_activeGoal->publish_feedback(feedback);
}


void PathFollower::abortMission(std::shared_ptr<GoalHandle>& gh,
    const std::string& reason)
{
    auto control_msg = geometry_msgs::msg::Point();
    control_msg.x = 0.0; control_msg.z = 0.0;
    control_msg.y = 0.0;

    m_pubControl->publish(control_msg);

    auto result = std::make_shared<car_interfaces::action::FollowPath::Result>();
    result->success = false;
    result->message = reason;
    gh->abort(result);
    m_active = false;
    saveTrajectory();
    RCLCPP_WARN(this->get_logger(), "Mission aborted: %s", reason.c_str());
}

std::vector<pp::Point> PathFollower::loadCSV(const std::string& path)
{
    std::vector<pp::Point> pts;
    std::ifstream file(path);
    if (!file.is_open())
        throw std::runtime_error("Cannot open CSV: " + path);

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#') continue;  // skip comments/header
        std::istringstream ss(line);
        std::string sx, sy;
        if (!std::getline(ss, sx, ',') || !std::getline(ss, sy, ',')) continue;
        pts.push_back({std::stod(sx), std::stod(sy)});
    }
    return pts;
}

void PathFollower::publishMarkers()
{
    visualization_msgs::msg::MarkerArray arr;

    // ...

    m_pubMarkers->publish(arr);
}

void PathFollower::recordPose()
{
    m_trajectory.push_back(m_pose);
}

void PathFollower::saveTrajectory()
{
    std::ofstream f(m_trajSavePath);
    if (!f.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Cannot write trajectory to %s", m_trajSavePath.c_str());
        return;
    }
    f << "# x,y,theta\n";
    for (const auto& p : m_trajectory)
    {
        f << p.x << "," << p.y << "," << p.theta << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory saved to %s (%zu poses)",
        m_trajSavePath.c_str(), m_trajectory.size());
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<PathFollower>(rclcpp::NodeOptions());
        rclcpp::spin(node);
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("path_follower"), "%s", e.what());
    }

    rclcpp::shutdown();

    std::cout << "[INFO] Shutting down path_follower node." << std::endl;

    return 0;
}

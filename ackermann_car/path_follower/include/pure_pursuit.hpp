//pure_pursuit.hpp
#pragma once

#include <iostream>
#include <optional>
#include <string>
#include <vector>

// #include <Eigen/Dense>

namespace pp 
{

// using Point = Eigen::Vector2d;
// using CarPose = Eigen::Vector3d;

struct Point
{
    double x;
    double y;
};

struct CarPose
{
    double x;
    double y;
    double theta;
};

// struct FeasibilityResult 
// {
//     bool ok;
//     std::string reason;
//     size_t failing_idx;
// };

class PPControl
{
public:
    PPControl(const double v, const double wheelbase, const double maxSteer, 
        const double lookaheadCoef, const double dt, std::vector<Point> &path);

    // FeasibilityResult validatePath() const;

    Point getNextPoint(const CarPose& pose, const double l_d) const;

    void update(const CarPose& pose);

    double getCmdSpeed() const { return m_cmdV; }
    double getCmdSteer() const { return m_lastSteer; }
    const Point& goalPoint() const { return m_path.back(); }

    bool pathEmpty() const { return m_path.empty(); }
    void dropPassedWaypoints(const CarPose& pose);

    void setMode(const std::string& mode, double kp=0.0, double ki=0.0, double kd=0.0);

private:
    // Controller mode
    std::string m_ctrlMode = "pure_pursuit"; // pure pursuit or pid

    double m_targetV = 0.0;
    double m_cmdV = 0.0;
    double m_wheelbase = 0.0;
    double m_lookaheadCoef = 0.0;
    double m_maxSteer = 0.0;
    double m_lastSteer = 0.0;
    double m_dt = 0.0;

    // PID variables
    double m_kp = 0.0;
    double m_ki = 0.0;
    double m_kd = 0.0;
    double m_integral = 0.0;
    double m_prevError = 0.0;

    std::vector<Point> m_path;

    const double MIN_TARGET_DIST = 0.01; // m
    const double MIN_SPEED = 0.1; // m/s
    const double MAX_STEER_VEL = 1.57; // rad/s
};

} // namespace pp
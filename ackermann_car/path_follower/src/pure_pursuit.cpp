#include <algorithm>
#include <math.h>

#include "pure_pursuit.hpp"

namespace pp 
{

PPControl::PPControl(const double v, const double wheelbase, const double maxSteer, 
        const double lookaheadCoef, const double dt, std::vector<Point> &path)
    : m_targetV(v), m_wheelbase(wheelbase), m_lookaheadCoef(lookaheadCoef), 
      m_maxSteer(maxSteer), m_dt(dt)
{
    m_path = path;
}

void PPControl::setMode(const std::string& mode, double kp, double ki, double kd)
{
    m_ctrlMode = mode;
    m_kp = kp; 
    m_ki = ki; 
    m_kd = kd;
    m_integral = 0.0; 
    m_prevError = 0.0;
}

Point PPControl::getNextPoint(const CarPose& pose, const double l_d) const
{
    // Find closest point on path
    size_t kMinDistance = 0;
    double min_distance = INFINITY;

    for (size_t k = 0; k < m_path.size(); ++k) 
    {
        double currDistance = std::hypot(pose.x - m_path[k].x, pose.y - m_path[k].y);

        if (currDistance < min_distance) 
        {
            min_distance = currDistance;
            kMinDistance = k; // index of min distance point
        }
    }

    size_t kTargetPoint = 0;
    double forwardDistance = 0.0;

    // Trying to find the closest point relative to lookahead distance
    for (kTargetPoint = kMinDistance; (forwardDistance < l_d && kTargetPoint < m_path.size()); ++kTargetPoint)
    {
        forwardDistance = std::hypot(pose.x - m_path[kTargetPoint].x,
                            pose.y - m_path[kTargetPoint].y);
    }

    if (kTargetPoint >= m_path.size()) return m_path.back();

    if (kTargetPoint > 0) --kTargetPoint;

    return m_path[kTargetPoint];
}

void PPControl::update(const CarPose& pose)
{
    if(!m_path.empty())
    {
        m_cmdV = m_targetV;

        // Compute lookahead
        double l_d = m_lookaheadCoef * std::max(m_cmdV, MIN_SPEED);

        auto target = getNextPoint(pose, l_d);

        double dx = target.x - pose.x;
        double dy = target.y - pose.y;

        double alpha = atan2(dy, dx) - pose.theta; // heading error
        alpha = atan2(sin(alpha), cos(alpha));

        if (m_ctrlMode == "pid")
        {
            double heading_error = alpha;
            m_integral += heading_error * m_dt;
            double derivative = (heading_error - m_prevError) / m_dt;
            m_prevError = heading_error;

            m_lastSteer = m_kp * heading_error + m_ki * m_integral + m_kd * derivative;
        }
        else // pure_pursuit
        {
            m_lastSteer = atan2(2.0 * m_wheelbase * sin(alpha), l_d);
        }

        m_lastSteer = std::clamp(m_lastSteer, -m_maxSteer, m_maxSteer);

        // Speed adjustment
        double curvature = fabs(tan(m_lastSteer) / m_wheelbase);

        double v_curv_limit = std::numeric_limits<double>::infinity();
        if (curvature > 1e-3) v_curv_limit = MAX_STEER_VEL / curvature;

        m_cmdV = std::min(m_cmdV, v_curv_limit);
        m_cmdV = std::max(m_cmdV, MIN_SPEED);
    }
    else
    {
        m_cmdV = 0.0;
        m_lastSteer = 0.0;
    }
}

void PPControl::dropPassedWaypoints(const CarPose& pose)
{
    while (m_path.size() > 1)
    {
        const double dist = std::hypot(pose.x - m_path.front().x, pose.y - m_path.front().y);
        const double dist_next = std::hypot(pose.x - m_path[1].x, pose.y - m_path[1].y);

        if (dist_next < dist && dist < 2.0 * m_lookaheadCoef * m_targetV)
            m_path.erase(m_path.begin());
        else
            break;
    }
}

} // namespace pp

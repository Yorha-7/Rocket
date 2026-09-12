#include "gnc/navigation.hpp"
#include <algorithm>
#include <stdexcept>

namespace navigation {

// ##### Constructor #####
// Goal: refuse an unsurvivable final target up front (not mid-flight --
// this is the one ground-safety check for the whole path, checked once
// against the REAL target, not a synthetic waypoint), lay out the
// straight-line waypoint list, and start the per-tick controller aimed
// at the first one.
Navigation::Navigation(const Eigen::Vector3d& final_target, double dt,
                       double kp, double ki, double kd)
    : step_(Eigen::Vector3d::Zero(), dt, kp, ki, kd) {
    if (final_target.z() <= MIN_TARGET_ALTITUDE_M) {
        throw std::invalid_argument(
            "Navigation target is on/near the ground (z=" + std::to_string(final_target.z()) +
            "m, minimum " + std::to_string(MIN_TARGET_ALTITUDE_M) +
            "m) -- aiming the nose there means aiming thrust into the ground.");
    }

    // Goal: "optimize for shortest distance" -- a straight line from the
    // pad toward the target. direction is the unit vector the user asked
    // for; WAYPOINT_STEP_M is the fixed magnitude along it. The last
    // waypoint is pinned to final_target exactly, so spacing rounding
    // never leaves the path short of (or past) the real target.
    direction_ = final_target.normalized();
    double total_distance = final_target.norm();
    int n_waypoints = std::max(1, static_cast<int>(total_distance / WAYPOINT_STEP_M));
    for (int k = 1; k < n_waypoints; ++k) {
        waypoints_.push_back(direction_ * (k * WAYPOINT_STEP_M));
    }
    waypoints_.push_back(final_target);

    step_.setTarget(waypoints_[current_waypoint_idx_]);
}

// ##### computeTvcTarget() #####
// Goal: drive the current waypoint's controller, then check whether the
// vehicle has closed in enough to advance to the next one -- everything
// else (PID, position fusion, the near-pad activation gate) lives in
// NavigationStep and isn't duplicated here.
Eigen::Vector3d Navigation::computeTvcTarget(const sensors::Gps& gps, const sensors::Gyro& gyro, double dt) {
    Eigen::Vector3d tvc_target = step_.computeTvcTarget(gps, gyro, dt);

    bool on_last_waypoint = (current_waypoint_idx_ + 1 == waypoints_.size());
    if (!on_last_waypoint) {
        Eigen::Vector3d position = step_.lastEstimatedPosition();
        double dist_to_waypoint = (position - waypoints_[current_waypoint_idx_]).norm();

        // Goal: a plain radius check alone can get permanently stuck --
        // measured directly on this project's own default target: real
        // closest approach to waypoint 1 was 2.7m, which is still a
        // miss against a 2m radius, and with nothing else able to move
        // current_waypoint_idx_ forward, the controller would keep
        // steering at that one early waypoint for the ENTIRE rest of the
        // flight (apogee measured dropping from ~2115m to ~560m in that
        // state). So also advance once the vehicle has flown PAST the
        // waypoint along the path's own straight-line direction,
        // regardless of how far off to the side it passed -- a fast or
        // wide-swinging flight then degrades to "advanced a little late"
        // instead of "frozen on one waypoint forever."
        bool passed_waypoint = (position - waypoints_[current_waypoint_idx_]).dot(direction_) > 0.0;

        if (dist_to_waypoint <= WAYPOINT_ARRIVAL_RADIUS_M || passed_waypoint) {
            ++current_waypoint_idx_;
            step_.setTarget(waypoints_[current_waypoint_idx_]);
        }
    }

    return tvc_target;
}

}  // namespace navigation

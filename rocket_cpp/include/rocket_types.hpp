#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

struct RocketState {
    Eigen::Vector3d position;     // x, y, z in NED frame (m)
    Eigen::Vector3d velocity;     // vx, vy, vz (m/s)
    Eigen::Vector3d orientation;  // roll, pitch, yaw (rad)
    Eigen::Vector3d angular_vel;  // p, q, r (rad/s)
    double mass;                  // mass (kg)
};

struct RocketParams {
    double cp_location;      // center of pressure (cm)
    double cg_location;      // center of gravity (cm) / initial mass (g)
    double I_xx, I_yy, I_zz; // moments of inertia (kg*m^2)
    double thrust_duration;  // burn time (s)
    double max_thrust;       // max thrust (N)
};

struct SimulationConfig {
    double launch_height;  // launch site height above ground (m)
    double init_tilt;      // initial pitch angle from vertical (degrees)
    double sim_duration;   // total simulation time (s)
    double dt;             // time step (s)
};
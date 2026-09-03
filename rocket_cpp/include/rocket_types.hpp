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

struct FlightConditions {
    double mach;           // Mach number
    double alpha;          // angle of attack (rad)
    double beta;           // sideslip angle (rad)
    double roll_rate;      // roll rate (rad/s)
    double pitch_rate;     // pitch rate (rad/s)
    double yaw_rate;       // yaw rate (rad/s)
    double altitude;       // m
    double velocity;       // m/s
    double reynolds;       // Reynolds number (based on body diameter)
    double dynamic_pressure; // Pa
    double reynolds_length; // Re based on body length
    double roll_angle;     // roll angle (rad)
};

struct AerodynamicCoefficients {
    double Cd;      // Total drag coefficient
    double Cn;      // Normal force coefficient
    double Ca;      // Axial force coefficient
    double Cm;      // Pitching moment coefficient
    double Cl;      // Lift coefficient
    double Cdp;     // Pressure drag coefficient
    double Cdf;     // Friction drag coefficient
    double Cdbase;  // Base drag coefficient
    double Cd_wave; // Wave drag coefficient
    double Cn_alpha; // Normal force derivative (per rad)
    double Cm_alpha; // Pitching moment derivative (per rad)
};

struct RocketParams {
    // Existing
    double cp_location;      // center of pressure (cm)
    double cg_location;      // center of gravity (cm)
    double I_xx, I_yy, I_zz; // moments of inertia (kg*m^2)
    double thrust_duration;  // burn time (s)
    double max_thrust;       // max thrust (N)
    
    // Body geometry
    double body_diameter;      // meters
    double body_length;        // meters  
    double nose_length;        // meters
    int nose_shape;            // 0=conical, 1=ogive, 2=hemisphere, 3=parabolic
    
    // Fin geometry
    int fin_count;             // number of fins (3, 4, etc.)
    double fin_span;           // meters (tip to root)
    double fin_root_chord;     // meters
    double fin_tip_chord;      // meters
    double fin_sweep;          // radians (leading edge sweep)
    double fin_thickness;      // meters (max thickness)
    double fin_cant;           // radians (cant angle)
    double fin_root_le_position; // distance from nose tip to fin root LE (m)
    
    // Body geometry
    double base_diameter;      // meters (base/boat-tail diameter)
    double boat_tail_length;   // meters
    double nose_radius;        // meters (for ogive/hemisphere)
    
    // Surface roughness
    double surface_roughness;  // meters (RMS roughness)
    
    // Reference area
    double reference_area;     // m^2 (for Cd normalization)
};

struct SimulationConfig {
    double launch_height;  // launch site height above ground (m)
    double init_tilt;      // initial pitch angle from vertical (degrees)
    double sim_duration;   // total simulation time (s)
    double dt;             // time step (s)
};

struct FlightConditions {
    double mach;           // Mach number
    double alpha;          // angle of attack (rad)
    double beta;           // sideslip angle (rad)
    double roll_rate;      // roll rate (rad/s)
    double pitch_rate;     // pitch rate (rad/s)
    double yaw_rate;       // yaw rate (rad/s)
    double altitude;       // m
    double velocity;       // m/s
    double reynolds;       // Reynolds number (based on body diameter)
    double dynamic_pressure; // Pa
    double reynolds_length; // Re based on body length
    double roll_angle;     // roll angle (rad)
};

struct AerodynamicCoefficients {
    double Cd;      // Total drag coefficient
    double Cn;      // Normal force coefficient
    double Ca;      // Axial force coefficient
    double Cm;      // Pitching moment coefficient
    double Cl;      // Lift coefficient
    double Cdp;     // Pressure drag coefficient
    double Cdf;     // Friction drag coefficient
    double Cdbase;  // Base drag coefficient
    double Cd_wave; // Wave drag coefficient
    double Cn_alpha; // Normal force derivative (per rad)
    double Cm_alpha; // Pitching moment derivative (per rad)
};

struct RocketParams {
    // Existing
    double cp_location;      // center of pressure (cm)
    double cg_location;      // center of gravity (cm)
    double I_xx, I_yy, I_zz; // moments of inertia (kg*m^2)
    double thrust_duration;  // burn time (s)
    double max_thrust;       // max thrust (N)
    
    // Body geometry
    double body_diameter;      // meters
    double body_length;        // meters  
    double nose_length;        // meters
    int nose_shape;            // 0=conical, 1=ogive, 2=hemisphere, 3=parabolic
    
    // Fin geometry
    int fin_count;             // number of fins (3, 4, etc.)
    double fin_span;           // meters (tip to root)
    double fin_root_chord;     // meters
    double fin_tip_chord;      // meters
    double fin_sweep;          // radians (leading edge sweep)
    double fin_thickness;      // meters (max thickness)
    double fin_cant;           // radians (cant angle)
    double fin_root_le_position; // distance from nose tip to fin root LE (m)
    
    // Body geometry
    double base_diameter;      // meters (base/boat-tail diameter)
    double boat_tail_length;   // meters
    double nose_radius;        // meters (for ogive/hemisphere)
    
    // Surface roughness
    double surface_roughness;  // meters (RMS roughness)
    
    // Reference area
    double reference_area;     // m^2 (for Cd normalization)
};

struct SimulationConfig {
    double launch_height;  // launch site height above ground (m)
    double init_tilt;      // initial pitch angle from vertical (degrees)
    double sim_duration;   // total simulation time (s)
    double dt;             // time step (s)
};

struct FlightConditions {
    double mach;           // Mach number
    double alpha;          // angle of attack (rad)
    double beta;           // sideslip angle (rad)
    double roll_rate;      // roll rate (rad/s)
    double pitch_rate;     // pitch rate (rad/s)
    double yaw_rate;       // yaw rate (rad/s)
    double altitude;       // m
    double velocity;       // m/s
    double reynolds;       // Reynolds number (based on body diameter)
    double dynamic_pressure; // Pa
    double reynolds_length; // Re based on body length
    double roll_angle;     // roll angle (rad)
};

struct AerodynamicCoefficients {
    double Cd;      // Total drag coefficient
    double Cn;      // Normal force coefficient
    double Ca;      // Axial force coefficient
    double Cm;      // Pitching moment coefficient
    double Cl;      // Lift coefficient
    double Cdp;     // Pressure drag coefficient
    double Cdf;     // Friction drag coefficient
    double Cdbase;  // Base drag coefficient
    double Cd_wave; // Wave drag coefficient
    double Cn_alpha; // Normal force derivative (per rad)
    double Cm_alpha; // Pitching moment derivative (per rad)
};
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

// ##### Why "struct" here #####
// Goal: hold data, nothing else. Every type below is just a labeled
// bundle of numbers passed between classes -- no behavior to protect,
// so "struct" (public fields) instead of "class" (private + getters).

// ##### RocketState #####
// Goal: capture everything the vehicle is doing at one instant -- where
// it is, how fast, which way it's pointed, how fast it's spinning, and
// how heavy it is right now. This is the one object that gets passed
// state -> next state through every physics step.
struct RocketState {
    Eigen::Vector3d position;     // x, y, z — z is altitude above the pad, positive up (m)
    Eigen::Vector3d velocity;     // vx, vy, vz (m/s)
    Eigen::Vector3d orientation;  // roll, pitch, yaw (rad)
    Eigen::Vector3d angular_vel;  // p, q, r (rad/s)
    double mass;                  // current vehicle mass (kg)

    // Goal: remember where the TVC nozzle physically is right now (not
    // where it was just told to go -- the actuator eases there, it
    // doesn't teleport). Stored here, not just inside the TVC class
    // itself, so replaying/logging a past step reads the SAME nozzle
    // angle that actually flew, not wherever the live actuator has
    // since moved on to.
    double gimbal_pitch_rad = 0.0;  // deflects nozzle toward +X, body X-Z plane
    double gimbal_yaw_rad = 0.0;    // deflects nozzle toward +Y, body Y-Z plane
};

// ##### FlightConditions #####
// Goal: snapshot everything the aerodynamics model needs to know about
// "right now" -- speed, altitude, angle relative to the air -- so drag
// and turning forces can be looked up from it in one place.
struct FlightConditions {
    double mach;              // Mach number
    double alpha;             // angle of attack (rad)
    double beta;               // sideslip angle (rad)
    double roll_rate;         // rad/s
    double pitch_rate;        // rad/s
    double yaw_rate;          // rad/s
    double altitude;          // m
    double velocity;          // airspeed (m/s)
    double reynolds;          // Reynolds number based on body diameter
    double dynamic_pressure;  // Pa
    double reynolds_length;   // Reynolds number based on body length
    double roll_angle;        // rad
};

// ##### AerodynamicCoefficients #####
// Goal: hold one complete set of drag/turning-force numbers for a given
// FlightConditions -- the output of AerodynamicsModel::computeCoefficients().
struct AerodynamicCoefficients {
    double Cd;       // total drag coefficient
    double Cn;       // normal force coefficient
    double Ca;       // axial force coefficient
    double Cm;       // pitching moment coefficient
    double Cl;       // lift coefficient
    double Cdp;      // pressure drag coefficient
    double Cdf;      // friction drag coefficient
    double Cdbase;   // base drag coefficient
    double Cd_wave;  // wave drag coefficient
    double Cn_alpha;  // normal force derivative (per rad)
    double Cm_alpha;  // pitching moment derivative (per rad)
};

// ##### RocketParams #####
// Goal: describe the rocket's fixed SHAPE and motor spec -- everything
// that never changes mid-flight. Anything that DOES change (mass, CG,
// CP, inertia, thrust) lives in FlightData/MassProperties instead, not
// here.
struct RocketParams {
    double thrust_duration;  // motor burn time (s)
    double max_thrust;       // peak thrust, informational only — actual thrust comes from FlightData (N)

    // Body geometry
    double body_diameter;    // m
    double body_length;      // m
    double nose_length;      // m
    int nose_shape;          // 0=conical, 1=ogive, 2=hemisphere, 3=parabolic

    // Fin geometry (one fin, repeated fin_count times around the body)
    int fin_count;
    double fin_span;             // root to tip (m)
    double fin_root_chord;       // m
    double fin_tip_chord;        // m
    double fin_sweep;            // leading-edge sweep distance (m)
    double fin_thickness;        // max thickness (m)
    double fin_cant;             // cant angle (rad)
    double fin_root_le_position; // distance from nose tip to fin root leading edge (m)

    // Base / boat-tail geometry
    double base_diameter;      // m
    double boat_tail_length;   // m
    double nose_radius;        // nose tip bluntness radius, 0 if sharp (m)

    // Surface finish
    double surface_roughness;  // RMS roughness (m)

    // Goal: precompute the cross-section area once (pi * radius^2) so
    // every drag/lift/moment coefficient in the sim normalizes against
    // the same number instead of recomputing it everywhere.
    double reference_area;     // m^2
};

// ##### SimulationConfig #####
// Goal: hold OUR OWN choices about how to run the integrator -- separate
// from anything the rocket's own design says. Two rockets with identical
// RocketParams could still be simulated with different launch tilts or
// time steps via this struct.
struct SimulationConfig {
    double launch_height;  // launch site height above ground (m)
    double init_tilt;      // initial pitch angle from vertical (degrees)
    double init_yaw;       // initial yaw angle (degrees) -- fixed for the whole
                            // flight, no restoring torque model yet (like roll)
    double sim_duration;   // total simulation time (s)
    double dt;              // time step (s)
};

// ##### FlightData #####
// Goal: carry the ONE thing this sim still trusts from OpenRocket's own
// data -- the motor's thrust and mass curve over time, resampled to a
// uniform dt. Everything else the sim needs (Cd, CP, CG, inertia) is
// computed independently from the vehicle's own geometry, not read here.
struct FlightData {
    std::vector<double> time;    // uniform timestep (s)
    std::vector<double> thrust;  // N
    std::vector<double> mass;    // g
    double dry_mass;             // mass at burnout (g)
};

// ##### MassProperties #####
// Goal: snapshot FlightData at one instant into exactly what the pitch
// and yaw torque equations need to know about the vehicle's current mass
// distribution (where its balance point is, how hard it resists spinning).
struct MassProperties {
    double cp_location_cm;
    double cg_location_cm;
    double I_xx;  // roll axis (kg*m^2)
    double I_yy;  // pitch axis (kg*m^2)
    double I_zz;  // yaw axis (kg*m^2) — assumed equal to I_yy (axisymmetric body)
};

// ##### PitchTorques / YawTorques #####
// Goal: break the total pitch/yaw torque down into its three named
// sources, purely for logging and plotting -- the integrator itself just
// uses the sum, but seeing gravity/aero/damping separately is what makes
// the CSV plots useful for debugging which one is actually driving a step.
struct PitchTorques {
    double gravity;      // N*m
    double aerodynamic;  // N*m
    double damping;      // N*m
};

// Same breakdown, yaw axis -- see YawTorques' mirror-of-pitch model in
// yaw_dynamics.cpp.
struct YawTorques {
    double gravity;      // N*m -- always 0, same reasoning as PitchTorques
    double aerodynamic;  // N*m
    double damping;      // N*m
};

// ##### gramsToKg #####
// Goal: one place to convert the .ork's own mass units (grams) into the
// kg this whole sim works in, so that conversion isn't repeated/risked
// at every call site.
inline double gramsToKg(double grams) { return grams / 1000.0; }

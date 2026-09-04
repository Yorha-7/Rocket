#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

// Everything the vehicle is doing right now: where it is, how fast,
// which way it's pointed, how fast it's rotating, and how heavy it is.
struct RocketState {
    Eigen::Vector3d position;     // x, y, z — z is altitude above the pad, positive up (m)
    Eigen::Vector3d velocity;     // vx, vy, vz (m/s)
    Eigen::Vector3d orientation;  // roll, pitch, yaw (rad)
    Eigen::Vector3d angular_vel;  // p, q, r (rad/s)
    double mass;                  // current vehicle mass (kg)
};

// Air-relative conditions the aerodynamics model needs at one instant.
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

// One set of aerodynamic force/moment coefficients, as produced by
// AerodynamicsModel::computeCoefficients() for a given FlightConditions.
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

// What the rocket physically *is* — its fixed shape and motor spec.
// Nothing here changes during flight; things that do (mass, CG, CP,
// inertia, thrust) live in FlightData/MassProperties instead.
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

    // Derived from body_diameter: pi * (body_diameter/2)^2. Used to
    // normalize every drag/lift/moment coefficient in the sim.
    double reference_area;     // m^2
};

// How we choose to run our own integrator — separate from anything the
// rocket's design implies.
struct SimulationConfig {
    double launch_height;  // launch site height above ground (m)
    double init_tilt;      // initial pitch angle from vertical (degrees)
    double sim_duration;   // total simulation time (s)
    double dt;              // time step (s)
};

// A flight's time history, resampled to a uniform dt. Thrust and mass are
// motor performance data -- there's no local motor database to derive them
// from independently, so they're the one thing still read from the .ork.
// Everything else the sim needs (Cd, CP, CG, inertia) is computed by
// AerodynamicsModel/VehicleMassModel from the vehicle's own geometry.
struct FlightData {
    std::vector<double> time;    // uniform timestep (s)
    std::vector<double> thrust;  // N
    std::vector<double> mass;    // g
    double dry_mass;             // mass at burnout (g)
};

// A snapshot of FlightData at one instant — everything the pitch-dynamics
// equations need to know about the vehicle's mass distribution right now.
struct MassProperties {
    double cp_location_cm;
    double cg_location_cm;
    double I_xx;  // roll axis (kg*m^2)
    double I_yy;  // pitch axis (kg*m^2)
    double I_zz;  // yaw axis (kg*m^2) — assumed equal to I_yy (axisymmetric body)
};

// The three torques that add up to the total pitch torque each step —
// broken out for logging/plotting, not used internally by the integrator.
struct PitchTorques {
    double gravity;      // N*m
    double aerodynamic;  // N*m
    double damping;      // N*m
};

// Convert grams to kg
inline double gramsToKg(double grams) { return grams / 1000.0; }

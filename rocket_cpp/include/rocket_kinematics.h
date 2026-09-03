#ifndef ROCKET_KINEMATICS_H
#define ROCKET_KINEMATICS_H

#include <Eigen/Dense>

struct RocketState {
    Eigen::Vector3d position;    // x, y, z in NED frame (m)
    Eigen::Vector3d velocity;    // vx, vy, vz (m/s)
    Eigen::Vector3d orientation; // roll, pitch, yaw (rad)
    Eigen::Vector3d angular_vel; // p, q, r (rad/s)
    double mass;                 // mass (kg)
};

struct RocketParams {
    double cp_location;     // center of pressure (cm)
    double cg_location;     // center of gravity (cm) / initial mass (g)
    double I_xx, I_yy, I_zz; // moments of inertia (kg*m^2)
    double thrust_duration; // burn time (s)
    double max_thrust;      // max thrust (N)
};

struct SimulationConfig {
    double launch_height;   // launch site height above ground (m)
    double init_tilt;       // initial pitch angle from vertical (degrees)
    double sim_duration;    // total simulation time (s)
    double dt;              // time step (s)
};

class RocketKinematics {
public:
    RocketKinematics(const RocketParams& params, const SimulationConfig& config);

    // Advance by one step
    RocketState step(const RocketState& state, double thrust,
                     double drag_coeff, double normal_coeff) const;

    // Full simulation run
    std::vector<RocketState> simulate(double time,
                                      const std::vector<double>& thrust_data,
                                      const std::vector<double>& drag_coeffs,
                                      const std::vector<double>& normal_coeffs) const;

    const SimulationConfig& getConfig() const { return config_; }
    const RocketParams& getParams() const { return params_; }

private:
    // Physical model
    Eigen::Vector3d computeAcceleration(const RocketState& state,
                                        double thrust,
                                        double drag_coeff,
                                        double normal_coeff) const;

    // Frame transformation
    Eigen::Matrix3d rocketToNedFrame(const RocketState& state) const;

    RocketParams params_;
    SimulationConfig config_;
};

// C interface functions
void rocket_to_ned_frame(const RocketState *state, Eigen::Matrix3d *R);
void compute_acceleration(const RocketState *state,
                          double thrust, double drag_coeff, double normal_coeff,
                          Eigen::Vector3d *accel);
void step_simulation(const RocketState *state, double thrust,
                    double drag_coeff, double normal_coeff,
                    double dt, RocketState *next);
int simulate_full(const RocketParams *params,
                  const SimulationConfig *config,
                  const double *thrust_data, int thrust_len,
                  const double *drag_coeffs, const double *normal_coeffs,
                  RocketState *states, int *n_out);

#endif
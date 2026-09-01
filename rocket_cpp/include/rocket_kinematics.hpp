#pragma once

#include "rocket_types.hpp"
#include <vector>

class RocketKinematics {
public:
    RocketKinematics(const RocketParams& params, const SimulationConfig& config);

    // Advance simulation by one time step
    RocketState step(const RocketState& state, double thrust,
                     double drag_coeff, double normal_coeff) const;

    // Run full simulation with ground termination
    std::vector<RocketState> simulate(double time,
                                      const std::vector<double>& thrust_data,
                                      const std::vector<double>& drag_coeffs,
                                      const std::vector<double>& normal_coeffs) const;

    const SimulationConfig& getConfig() const { return config_; }
    const RocketParams& getParams() const { return params_; }

private:
    Eigen::Vector3d computeAcceleration(const RocketState& state,
                                        double thrust,
                                        double drag_coeff,
                                        double normal_coeff) const;
    Eigen::Matrix3d rocketToNedFrame(const RocketState& state) const;

    RocketParams params_;
    SimulationConfig config_;
};

// Convert grams to kg
inline double gramsToKg(double grams) { return grams / 1000.0; }
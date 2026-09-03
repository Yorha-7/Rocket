#pragma once

#include "rocket_types.hpp"
#include "csv_loader.hpp"
#include <vector>

class RocketKinematics {
public:
    RocketKinematics(const RocketParams& params, const SimulationConfig& config);

    // Advance simulation by one time step
    RocketState step(const RocketState& state, double thrust,
                     double drag_coeff) const;

    // Run full simulation with ground termination using FlightData (includes real mass)
    std::vector<RocketState> simulate(double time, const FlightData& flight_data) const;

    const SimulationConfig& getConfig() const { return config_; }
    const RocketParams& getParams() const { return params_; }

private:
    Eigen::Vector3d computeAcceleration(const RocketState& state,
                                        double thrust,
                                        double drag_coeff) const;
    Eigen::Matrix3d rocketToNedFrame(const RocketState& state) const;

    // Pitch dynamics - modular methods for each component
    double computeGravityTorque(const RocketState& state) const;
    double computeAerodynamicMoment(const RocketState& state) const;
    double computeDampingTorque(const RocketState& state) const;
    double computeTotalPitchTorque(const RocketState& state) const;
    double computePitchAcceleration(double total_torque) const;
    void updatePitchDynamics(RocketState& next, const RocketState& state) const;
    
    // Aerodynamic coefficient calculations
    double calculateCnAlpha() const;

    RocketParams params_;
    SimulationConfig config_;
};

// Convert grams to kg
inline double gramsToKg(double grams) { return grams / 1000.0; }
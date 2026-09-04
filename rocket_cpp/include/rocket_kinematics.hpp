#pragma once

#include "rocket_types.hpp"
#include "aerodynamics.hpp"
#include "mass_properties_model.hpp"
#include "ork_mass_components.hpp"
#include <vector>

// The physics engine: turns a thrust/mass history into a full trajectory,
// computing its own drag/normal-force coefficients (AerodynamicsModel) and
// its own mass properties (VehicleMassModel) from the vehicle's geometry
// at every step -- nothing is read from a pre-solved simulation. Methods
// split across rocket_kinematics.cpp (translation + top-level loop) and
// pitch_dynamics.cpp (pitch-axis torque model).
class RocketKinematics {
public:
    RocketKinematics(const RocketParams& params, const SimulationConfig& config,
                     const std::vector<MassComponent>& mass_components);

    // Advance simulation by one time step. state.mass is this instant's
    // vehicle mass (structure + whatever motor propellant remains); thrust
    // is this instant's motor thrust. Everything else -- drag, CP, CG,
    // inertia -- is computed internally.
    RocketState step(const RocketState& state, double thrust) const;

    // Run the full flight, stepping through FlightData until ground contact.
    std::vector<RocketState> simulate(double time, const FlightData& flight_data) const;

    const SimulationConfig& getConfig() const { return config_; }
    const RocketParams& getParams() const { return params_; }

    // Diagnostics for logging/plotting: recompute the net world-frame
    // force and the pitch-torque breakdown for an already-simulated
    // state. Not used by step() itself -- just the same physics, exposed
    // so main.cpp can log what actually drove each step.
    Eigen::Vector3d computeNetForce(const RocketState& state, double thrust) const;
    PitchTorques computePitchTorques(const RocketState& state) const;

private:
    // ---- Translational motion (rocket_kinematics.cpp) ----
    Eigen::Vector3d computeAcceleration(const RocketState& state, double thrust) const;
    Eigen::Matrix3d rocketToNedFrame(const RocketState& state) const;
    FlightConditions buildFlightConditions(const RocketState& state) const;

    // ---- Pitch dynamics (pitch_dynamics.cpp) ----
    double computeGravityTorque(const RocketState& state, const MassProperties& mp) const;
    double computeAerodynamicMoment(const RocketState& state, const MassProperties& mp) const;
    double computeDampingTorque(const RocketState& state, const MassProperties& mp) const;
    double computeTotalPitchTorque(const RocketState& state, const MassProperties& mp) const;
    double computePitchAcceleration(double total_torque, const MassProperties& mp) const;
    void updatePitchDynamics(RocketState& next, const RocketState& state) const;

    RocketParams params_;
    SimulationConfig config_;
    AerodynamicsModel aero_;
    VehicleMassModel mass_model_;
    double cp_location_cm_;  // Barrowman CP is geometry-only -- computed once, cached
};

#pragma once

#include "sim/rocket_types.hpp"
#include "sim/aerodynamics.hpp"
#include "sim/mass_properties_model.hpp"
#include "ork/ork_mass_components.hpp"
#include "gnc/thrust_vector_control.hpp"
#include "gnc/navigation.hpp"
#include <vector>

// The physics engine: turns a thrust/mass history into a full trajectory,
// computing its own drag/normal-force coefficients (AerodynamicsModel) and
// its own mass properties (VehicleMassModel) from the vehicle's geometry
// at every step -- nothing is read from a pre-solved simulation. Methods
// split across rocket_kinematics.cpp (translation + top-level loop),
// pitch_dynamics.cpp (pitch-axis torque model), and yaw_dynamics.cpp
// (yaw-axis torque model -- the exact mirror of pitch, one plane over).
class RocketKinematics {
public:
    RocketKinematics(const RocketParams& params, const SimulationConfig& config,
                     const std::vector<MassComponent>& mass_components);

    // Advance simulation by one time step. state.mass is this instant's
    // vehicle mass (structure + whatever motor propellant remains); thrust
    // is this instant's motor thrust. tvc_target_dir is where the TVC
    // actuators are being commanded to point the FORCE this step (body
    // frame, need not be normalized; default (0,0,1) = no deflection
    // commanded). The actuators don't jump there -- this call also
    // advances the live TVC actuator state by one dt and bakes the result
    // into next.gimbal_pitch_rad/gimbal_yaw_rad, which is what actually
    // drives thrust direction (see computeNetForce). Everything else --
    // drag, CP, CG, inertia -- is computed internally.
    //
    // Not const: advancing the actuators is real state change, not just a
    // read. Must be called in sequence (as simulate() does) -- the live
    // TVC object always represents wherever `state`'s own gimbal angles
    // say it is, so calling this out of order desyncs the two.
    RocketState step(const RocketState& state, double thrust,
                      const Eigen::Vector3d& tvc_target_dir = Eigen::Vector3d(0, 0, 1));

    // Run the full flight, stepping through FlightData until ground
    // contact. tvc_targets is an optional per-step TVC command (same
    // indexing as flight_data); shorter than the flight, or omitted
    // entirely, and the remaining/all steps command no deflection.
    //
    // If navigation is non-null, it OVERRIDES tvc_targets: each step,
    // fresh Gps/Gyro readings are built from the state that step just
    // produced and handed to navigation->computeTvcTarget() to get that
    // step's command instead. This is the only place Navigation/Sensors
    // touch the integrator -- computeNetForce/step themselves know
    // nothing about guidance, only about whatever direction they're told.
    std::vector<RocketState> simulate(double time, const FlightData& flight_data,
                                       const std::vector<Eigen::Vector3d>& tvc_targets = {},
                                       Navigation* navigation = nullptr);

    // Return type is a const reference (a read-only alias to the member,
    // no copy made) -- cheap, but the caller can't modify config_/params_
    // through it.
    const SimulationConfig& getConfig() const { return config_; }
    const RocketParams& getParams() const { return params_; }

    // Diagnostics for logging/plotting: recompute the net world-frame
    // force and the pitch-torque breakdown for an already-simulated
    // state. Not used by step() itself -- just the same physics, exposed
    // so main.cpp can log what actually drove each step.
    Eigen::Vector3d computeNetForce(const RocketState& state, double thrust) const;
    PitchTorques computePitchTorques(const RocketState& state) const;
    YawTorques computeYawTorques(const RocketState& state) const;

    // Body-to-world rotation matrix (see the .cpp for the actual DCM).
    // Public and static -- it's a pure function of orientation, no
    // instance state involved -- so Sensors/Navigation can reuse the
    // exact same formula instead of a second hand-copied version drifting
    // out of sync with this one (see README's cos(theta) bug history for
    // why a second copy is worth avoiding).
    static Eigen::Matrix3d rocketToNedFrame(const RocketState& state);

private:
    // ---- Translational motion (rocket_kinematics.cpp) ----
    Eigen::Vector3d computeAcceleration(const RocketState& state, double thrust) const;
    FlightConditions buildFlightConditions(const RocketState& state) const;

    // ---- Pitch dynamics (pitch_dynamics.cpp) ----
    double computeGravityTorque(const RocketState& state, const MassProperties& mp) const;
    double computeAerodynamicMoment(const RocketState& state, const MassProperties& mp) const;
    double computeDampingTorque(const RocketState& state, const MassProperties& mp) const;
    double computeTotalPitchTorque(const RocketState& state, const MassProperties& mp) const;
    double computePitchAcceleration(double total_torque, const MassProperties& mp) const;
    void updatePitchDynamics(RocketState& next, const RocketState& state) const;

    // ---- Yaw dynamics (yaw_dynamics.cpp) -- exact mirror of the above ----
    double computeYawGravityTorque(const RocketState& state, const MassProperties& mp) const;
    double computeYawAeroMoment(const RocketState& state, const MassProperties& mp) const;
    double computeYawDampingTorque(const RocketState& state, const MassProperties& mp) const;
    double computeTotalYawTorque(const RocketState& state, const MassProperties& mp) const;
    double computeYawAcceleration(double total_torque, const MassProperties& mp) const;
    void updateYawDynamics(RocketState& next, const RocketState& state) const;

    // These are stored by value (real, owned copies -- not references),
    // unlike AerodynamicsModel::params_ which stores a reference. That's
    // why this class's constructor doesn't need "explicit": it takes 3
    // arguments, so it's never eligible for the single-argument implicit
    // conversion explicit guards against.
    RocketParams params_;
    SimulationConfig config_;
    AerodynamicsModel aero_;
    VehicleMassModel mass_model_;
    double cp_location_cm_;  // Barrowman CP is geometry-only -- computed once, cached

    // The live TVC actuator: only step() touches this, always in lockstep
    // with whatever state it was just handed (see step()'s comment).
    // computeNetForce() deliberately does NOT read this -- it reads the
    // gimbal angles already baked into the RocketState it's given.
    ThrustVectorControl tvc_;
};

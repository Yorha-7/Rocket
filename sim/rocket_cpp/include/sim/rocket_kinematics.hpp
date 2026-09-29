#pragma once

#include "sim/rocket_types.hpp"
#include "sim/aerodynamics.hpp"
#include "sim/mass_properties_model.hpp"
#include "ork/ork_mass_components.hpp"
#include "gnc/thrust_vector_control.hpp"
#include "gnc/navigation.hpp"
#include <vector>

// ##### RocketKinematics #####
// Goal: the physics engine. Feed it a thrust/mass history and it plays
// out a full trajectory, working out its own drag/turning coefficients
// (AerodynamicsModel) and its own mass properties (VehicleMassModel)
// from the vehicle's geometry at every step -- nothing here is read from
// a pre-solved simulation. Methods are split across three files:
// rocket_kinematics.cpp (translation + the top-level loop),
// pitch_dynamics.cpp (how the nose tips forward/back), and
// yaw_dynamics.cpp (how the nose swings side to side -- a mirror of pitch).
class RocketKinematics {
public:
    RocketKinematics(const RocketParams& params, const SimulationConfig& config,
                     const std::vector<MassComponent>& mass_components);

    // ##### step() #####
    // Goal: advance the simulation by exactly one time step. Give it the
    // vehicle's current state, this instant's motor thrust, and which
    // way TVC is being commanded to point the force -- it works out drag,
    // gravity, and turning, and hands back the state one dt later. Also
    // advances the TVC actuator itself by one dt (it eases toward a
    // command, it doesn't teleport there), so calls must happen in order
    // -- skipping around would desync the live actuator from the state
    // it's supposed to represent.
    RocketState step(const RocketState& state, double thrust,
                      const Eigen::Vector3d& tvc_target_dir = Eigen::Vector3d(0, 0, 1));

    // ##### simulate() #####
    // Goal: run step() over and over until the vehicle hits the ground,
    // building the full flight history. tvc_targets is an optional fixed
    // command sequence; if a Navigation object is given instead, it
    // OVERRIDES that sequence -- each step, fresh simulated Gps/Gyro
    // readings get built and handed to Navigation to decide the command
    // live. This is the only place guidance ever touches the physics;
    // step() itself has no idea whether a human, a script, or an AI
    // guidance law chose the direction it was given.
    std::vector<RocketState> simulate(double time, const FlightData& flight_data,
                                       const std::vector<Eigen::Vector3d>& tvc_targets = {},
                                       navigation::Navigation* navigation = nullptr);

    // Goal: hand back read-only views of this run's own settings --
    // callers can inspect them but never accidentally mutate them
    // through this reference.
    const SimulationConfig& getConfig() const { return config_; }
    const RocketParams& getParams() const { return params_; }

    // ##### Diagnostics #####
    // Goal: let main.cpp recompute the exact same force/torque numbers
    // step() used internally, purely for logging/plotting -- these three
    // don't feed back into the simulation itself.
    Eigen::Vector3d computeNetForce(const RocketState& state, double thrust) const;
    PitchTorques computePitchTorques(const RocketState& state) const;
    YawTorques computeYawTorques(const RocketState& state) const;

    // ##### rocketToNedFrame() #####
    // Goal: build the rotation matrix that turns "the rocket's own
    // directions" into "the world's directions" for a given orientation.
    // Public and static (it only depends on the orientation passed in,
    // nothing about this specific vehicle instance) so Sensors and
    // Navigation can reuse the EXACT same formula instead of each
    // keeping a hand-copied version that could quietly drift out of sync.
    static Eigen::Matrix3d rocketToNedFrame(const RocketState& state);

private:
    // ##### Translational motion (rocket_kinematics.cpp) #####
    Eigen::Vector3d computeAcceleration(const RocketState& state, double thrust) const;
    FlightConditions buildFlightConditions(const RocketState& state) const;

    // ##### Pitch dynamics (pitch_dynamics.cpp) #####
    double computeGravityTorque(const RocketState& state, const MassProperties& mp) const;
    double computeAerodynamicMoment(const RocketState& state, const MassProperties& mp) const;
    double computeDampingTorque(const RocketState& state, const MassProperties& mp) const;
    double computeTotalPitchTorque(const RocketState& state, const MassProperties& mp) const;
    double computePitchAcceleration(double total_torque, const MassProperties& mp) const;
    void updatePitchDynamics(RocketState& next, const RocketState& state) const;

    // ##### Yaw dynamics (yaw_dynamics.cpp) -- exact mirror of pitch #####
    double computeYawGravityTorque(const RocketState& state, const MassProperties& mp) const;
    double computeYawAeroMoment(const RocketState& state, const MassProperties& mp) const;
    double computeYawDampingTorque(const RocketState& state, const MassProperties& mp) const;
    double computeTotalYawTorque(const RocketState& state, const MassProperties& mp) const;
    double computeYawAcceleration(double total_torque, const MassProperties& mp) const;
    void updateYawDynamics(RocketState& next, const RocketState& state) const;

    // ##### What this instance remembers between calls #####
    // Goal: params_/config_ are owned copies (not references) so this
    // object stays valid even if the caller's originals go away --
    // that's also why the constructor doesn't need "explicit": taking 3
    // arguments already rules it out of the single-argument implicit-
    // conversion case "explicit" guards against.
    RocketParams params_;
    SimulationConfig config_;
    AerodynamicsModel aero_;
    VehicleMassModel mass_model_;
    double cp_location_cm_;  // Barrowman CP is geometry-only -- computed once, cached

    // Goal: keep one live TVC actuator alive across steps, since a real
    // servo's position is memory, not something re-derived each call.
    // Only step() touches it. computeNetForce() deliberately reads the
    // gimbal angles already baked into RocketState instead, not this --
    // so logging/replaying a past step reads history, not wherever the
    // live actuator has moved on to since.
    ThrustVectorControl tvc_;

    // Goal: track how far into the flight step() has gotten, purely so
    // updatePitchDynamics() can tell whether it's still inside the brief
    // ignition-transient window its safety clamp is scoped to (see
    // pitch_dynamics.cpp) -- RocketState itself carries no time field.
    double elapsed_time_s_ = 0.0;
};

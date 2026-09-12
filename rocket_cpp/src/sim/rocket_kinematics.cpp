#include "sim/rocket_kinematics.hpp"
#include "gnc/sensors.hpp"
#include <Eigen/Dense>
#include <cmath>

RocketKinematics::RocketKinematics(const RocketParams& params, const SimulationConfig& config,
                                   const std::vector<MassComponent>& mass_components)
    : params_(params), config_(config), aero_(params_),
      mass_model_(mass_components, params_.body_diameter, params_.body_length),
      cp_location_cm_(aero_.computeCenterOfPressure()) {}

// ##### rocketToNedFrame() #####
// Goal: given the vehicle's current roll/pitch/yaw, build the matrix
// that converts "a direction described in the rocket's own axes" into
// "that same direction described in world axes" (body -> world). Used
// any time a force acts along the rocket's own body (like thrust) and
// needs to be expressed in world coordinates to be added to gravity/drag
// and integrated into position. Standard aerospace Z-Y-X convention:
// yaw first, then pitch, then roll.
Eigen::Matrix3d RocketKinematics::rocketToNedFrame(const RocketState& state) {
    double roll  = state.orientation(0);
    double pitch = state.orientation(1);
    double yaw   = state.orientation(2);

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    Eigen::Matrix3d R;
    R << cy*cp,  cy*sp*sr - sy*cr,   cy*sp*cr + sy*sr,
         sy*cp,  sy*sp*sr + cy*cr,   sy*sp*cr - cy*sr,
         -sp,    cp*sr,              cp*cr;
    return R;
}

// ##### buildFlightConditions() #####
// Goal: package up "what the air looks like to the vehicle right now" --
// how fast, how high, and how far off-axis the airflow is hitting it
// (angle of attack in the pitch plane, sideslip in the yaw plane) -- into
// one bundle the aerodynamics model and both torque models can share.
//
// Alpha/beta are found by rotating the WORLD-frame velocity into BODY
// frame (via rocketToNedFrame's transpose, since going world->body is
// the reverse of body->world) rather than assuming the velocity stays in
// some fixed plane -- needed now that TVC can push the vehicle off-axis
// in either plane independently.
FlightConditions RocketKinematics::buildFlightConditions(const RocketState& state) const {
    double altitude = std::max(0.0, state.position(2));
    double velocity = state.velocity.norm();

    FlightConditions fc{};
    fc.altitude = altitude;
    fc.velocity = velocity;
    fc.mach = velocity / aero_.getSpeedOfSound(altitude);
    fc.dynamic_pressure = 0.5 * aero_.getDensity(altitude) * velocity * velocity;

    // Goal: alpha must read POSITIVE when the nose is leading the
    // velocity vector, so the restoring torque (-Cn_alpha*alpha*d) comes
    // out negative and pulls the nose back toward the airflow -- verified
    // against a concrete case (nose tipped +10deg, purely vertical
    // velocity -> alpha should read +10deg). The un-negated form gives
    // the opposite sign, which is what made pitch run away instead of
    // settling the first time this was written -- see README history.
    Eigen::Vector3d v_body = rocketToNedFrame(state).transpose() * state.velocity;
    if (velocity > 1e-6) {
        fc.alpha = atan2(-v_body.x(), v_body.z());  // body X-Z plane, vs. nose axis (Z)
        fc.beta = atan2(-v_body.y(), v_body.z());   // body Y-Z plane, vs. nose axis (Z)
    } else {
        fc.alpha = 0.0;
        fc.beta = 0.0;
    }
    fc.roll_rate = state.angular_vel(0);
    fc.pitch_rate = state.angular_vel(1);
    fc.yaw_rate = state.angular_vel(2);
    fc.roll_angle = state.orientation(0);
    return fc;
}

// ##### computeNetForce() #####
// Goal: add up every force acting on the rocket -- thrust, drag,
// gravity -- all expressed in WORLD frame, since that's the frame
// position/velocity are integrated in.
Eigen::Vector3d RocketKinematics::computeNetForce(const RocketState& state, double thrust) const {
    const double g0 = 9.80665;
    const double v = state.velocity.norm();
    const double v2 = v * v;

    // Goal: get the thrust push in BODY frame first (which way the
    // nozzle is actually deflected, read from state -- not the live TVC
    // object, see tvc_'s declaration comment -- so replaying an old step
    // reads that step's real history).
    Eigen::Vector3d thrust_body = -thrust * ThrustVectorControl::nozzleDirectionFromAngles(
        state.gimbal_pitch_rad, state.gimbal_yaw_rad);

    double altitude = std::max(0.0, state.position(2));
    double rho = aero_.getDensity(altitude);

    FlightConditions fc = buildFlightConditions(state);
    double drag_coeff = aero_.computeCoefficients(fc).Cd;
    double drag_area = params_.reference_area;

    // Goal: drag straight-up opposes whatever direction the vehicle is
    // actually moving through the air -- built directly from
    // state.velocity, which is ALREADY a world-frame quantity (see
    // RocketState's own doc comment), so this is already the answer, no
    // conversion needed. (A stale version of this file used to rotate
    // this through rocketToNedFrame() a second time -- meaningless, since
    // it was never in body frame to begin with, and the double rotation
    // could flip drag into effectively pushing the rocket during a fast
    // attitude change. See README Staging Notes for the fix writeup.)
    Eigen::Vector3d drag_ned;
    if (v > 1e-6) {
        drag_ned = -state.velocity.normalized() * drag_coeff * drag_area * 0.5 * rho * v2;
    } else {
        drag_ned = Eigen::Vector3d::Zero();
    }

    // Goal: NOW convert the body-frame thrust push into world frame --
    // this rotation genuinely is needed, since thrust_body really is
    // expressed along the vehicle's own axes.
    Eigen::Matrix3d R = rocketToNedFrame(state);
    Eigen::Vector3d thrust_ned = R * thrust_body;

    Eigen::Vector3d gravity(0, 0, -g0 * state.mass);

    return thrust_ned + drag_ned + gravity;
}

Eigen::Vector3d RocketKinematics::computeAcceleration(const RocketState& state, double thrust) const {
    return computeNetForce(state, thrust) / state.mass;
}

// ##### step() #####
// Goal: advance one dt -- integrate translation (Newton's second law:
// force -> acceleration -> velocity -> position), hand off to the pitch
// and yaw torque models for rotation, and advance the TVC actuator so
// next call's force reads its updated (lagged, not instant) position.
RocketState RocketKinematics::step(const RocketState& state, double thrust,
                                   const Eigen::Vector3d& tvc_target_dir) {
    // Goal: tell the actuator where it's being commanded to go THIS
    // step, but use the force from its position BEFORE that command --
    // it hasn't physically moved yet, matching a real servo's lag.
    tvc_.commandForceDirection(tvc_target_dir);
    elapsed_time_s_ += config_.dt;

    RocketState next = state;

    Eigen::Vector3d accel = computeAcceleration(state, thrust);

    next.velocity = state.velocity + accel * config_.dt;
    next.position = state.position + next.velocity * config_.dt;

    updatePitchDynamics(next, state);
    updateYawDynamics(next, state);

    // Goal: roll has no torque model yet (see README Staging Notes) --
    // just keep its angle wrapped into a sane range, don't let it drift
    // unbounded.
    next.orientation(0) = fmod(state.orientation(0), 2*M_PI);

    // Goal: now that this step's force has already been computed against
    // the PRE-command actuator position, actually move the actuator
    // toward its (possibly just-changed) target, and save where it ended
    // up -- that's what computeNetForce reads on the FOLLOWING call.
    tvc_.step(config_.dt);
    next.gimbal_pitch_rad = tvc_.currentGimbalPitchRad();
    next.gimbal_yaw_rad = tvc_.currentGimbalYawRad();

    return next;
}

// ##### simulate() #####
// Goal: run step() repeatedly to play out a full flight, from launch to
// ground contact, feeding it thrust/mass from FlightData and (optionally)
// a live guidance decision from Navigation each step.
std::vector<RocketState> RocketKinematics::simulate(double time, const FlightData& flight_data,
                                                    const std::vector<Eigen::Vector3d>& tvc_targets,
                                                    navigation::Navigation* navigation) {
    // Goal: size the loop from the requested flight time alone -- NOT
    // clamped to flight_data's own length. That array only covers
    // however long OpenRocket's own (unguided) simulation happened to
    // run, which can be well short of how long THIS flight actually
    // takes once TVC pushes it onto a different trajectory. Ground
    // contact (below) is still what actually ends the loop early; this
    // just stops it from running out of thrust/mass data first and
    // silently stopping mid-air, still hundreds of meters up.
    int n_steps = static_cast<int>(time / config_.dt);
    std::vector<RocketState> states(n_steps + 1);

    // Goal: set up the vehicle exactly as it sits on the pad at t=0.
    RocketState initial;
    initial.position = Eigen::Vector3d(0, 0, config_.launch_height);
    initial.velocity = Eigen::Vector3d::Zero();
    initial.orientation = Eigen::Vector3d(0,
        config_.init_tilt * M_PI / 180.0,
        config_.init_yaw * M_PI / 180.0);
    initial.angular_vel = Eigen::Vector3d::Zero();
    initial.mass = gramsToKg(flight_data.mass.front());
    initial.gimbal_pitch_rad = 0.0;
    initial.gimbal_yaw_rad = 0.0;
    states[0] = initial;

    const Eigen::Vector3d no_deflection(0, 0, 1);
    sensors::Gps nav_gps;
    sensors::Gyro nav_gyro;

    for (int i = 0; i < n_steps; ++i) {
        // Goal: past the end of the recorded flight data, the motor is
        // done burning (thrust=0) and there's no more propellant left to
        // shed (mass holds at its dry value) -- real motors don't
        // restart or un-burn fuel. Same defensive indexing main.cpp's
        // own force-recomputation loop already uses.
        double thrust = (i < (int)flight_data.thrust.size()) ? flight_data.thrust[i] : 0.0;
        double mass_kg = (i < (int)flight_data.mass.size())
            ? gramsToKg(flight_data.mass[i])
            : gramsToKg(flight_data.dry_mass);

        // Goal: work out which way to command TVC this step -- either a
        // fixed pre-scripted sequence, or (if guidance is attached) a
        // live decision from fresh simulated sensor readings of the
        // state THIS step is starting from (never a future/lookahead state).
        Eigen::Vector3d tvc_target;
        if (navigation != nullptr) {
            nav_gps.update(states[i], config_.dt);
            const RocketState& prev = (i > 0) ? states[i - 1] : states[i];
            nav_gyro.update(states[i], prev, config_.dt);
            tvc_target = navigation->computeTvcTarget(nav_gps, nav_gyro, config_.dt);
        } else {
            tvc_target = (i < (int)tvc_targets.size()) ? tvc_targets[i] : no_deflection;
        }

        RocketState temp = states[i];
        temp.mass = mass_kg;
        states[i + 1] = step(temp, thrust, tvc_target);

        // Goal: stop the flight the instant it crosses the ground,
        // landing exactly ON z=0 instead of overshooting into negative
        // altitude -- find how far between this step and the last one
        // the crossing actually happened (frac), and linearly interpolate
        // every state variable back to that exact instant.
        if (states[i + 1].position(2) < 0.0) {
            double z_above = states[i].position(2);
            double z_below = states[i + 1].position(2);
            if (z_above > 0 && z_below < 0) {
                double frac = z_above / (z_above - z_below);
                states[i + 1].position = states[i].position +
                    (states[i + 1].position - states[i].position) * frac;
                states[i + 1].position(2) = 0.0;
                states[i + 1].velocity = states[i].velocity +
                    (states[i + 1].velocity - states[i].velocity) * frac;
                states[i + 1].orientation = states[i].orientation +
                    (states[i + 1].orientation - states[i].orientation) * frac;
                states[i + 1].angular_vel = states[i].angular_vel +
                    (states[i + 1].angular_vel - states[i].angular_vel) * frac;
                states[i + 1].velocity.setZero();
                states[i + 1].orientation.setZero();
                states[i + 1].angular_vel.setZero();
                states.resize(i + 2);
                return states;
            }
        }
    }
    return states;
}

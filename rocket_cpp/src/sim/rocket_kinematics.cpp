#include "sim/rocket_kinematics.hpp"
#include "gnc/sensors.hpp"
#include <Eigen/Dense>
#include <cmath>

RocketKinematics::RocketKinematics(const RocketParams& params, const SimulationConfig& config,
                                   const std::vector<MassComponent>& mass_components)
    : params_(params), config_(config), aero_(params_),
      mass_model_(mass_components, params_.body_diameter, params_.body_length),
      cp_location_cm_(aero_.computeCenterOfPressure()) {}

// Body-to-world rotation matrix, built from the current roll/pitch/yaw
// Euler angles: the standard Z-Y-X (yaw, then pitch, then roll) aerospace
// DCM, R = Rz(yaw)*Ry(pitch)*Rx(roll). Used to turn forces that act along
// the rocket's own axes (thrust) into forces in the world frame we
// integrate position/velocity in -- e.g. a pitched-over rocket's thrust
// should pick up a horizontal component, which this matrix now does.
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

// What the aerodynamics model needs to know about "right now": how fast,
// how high, at what angle of attack. Shared by translation (drag) and
// pitch dynamics (normal force), so both use the same flight condition.
// reynolds/reynolds_length are left at 0 here -- computeFrictionDrag
// works out its own Reynolds number internally (it needs the choice of
// reference length, nose+body, which isn't this struct's job to know).
FlightConditions RocketKinematics::buildFlightConditions(const RocketState& state) const {
    double altitude = std::max(0.0, state.position(2));
    double velocity = state.velocity.norm();

    FlightConditions fc{};
    fc.altitude = altitude;
    fc.velocity = velocity;
    fc.mach = velocity / aero_.getSpeedOfSound(altitude);
    fc.dynamic_pressure = 0.5 * aero_.getDensity(altitude) * velocity * velocity;

    // Real angle of attack (alpha) and sideslip (beta): the angle between
    // the body's own nose axis and its actual velocity, decomposed into
    // the pitch plane (alpha) and yaw plane (beta). Computed by rotating
    // velocity into BODY frame rather than assuming it stays in some
    // fixed plane -- that assumption held back when yaw was always
    // static and nothing could push velocity sideways out of it, but TVC
    // can now steer independently in both planes, so alpha/beta need to
    // be exact regardless of where the velocity vector actually points.
    // At v=0 (pad, or a momentary stall) both come out 0, same as before.
    //
    // Negated (atan2(-x,z), not atan2(x,z)): alpha needs to be POSITIVE
    // when the nose leads the velocity vector (so the restoring torque
    // -Cn_alpha*alpha*d comes out negative and pulls the nose back) --
    // verified against the old, already-correct single-plane formula on
    // a concrete case (nose tipped +10deg, purely vertical velocity
    // should give alpha=+10deg). The un-negated form gives exactly the
    // opposite sign, which is what made pitch run away to the clamp
    // instead of settling the first time this was written.
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

// Everything that pushes or pulls on the rocket: thrust along the
// TVC-deflected nozzle direction (state's own gimbal_pitch_rad/
// gimbal_yaw_rad -- see ThrustVectorControl; (0,0,thrust) when both are
// 0, i.e. no deflection), drag opposing its velocity (drag coefficient
// computed by our own AerodynamicsModel, not read from anywhere), and
// gravity -- summed in the world frame. computeAcceleration() just
// divides this by mass; computeNetForce() exposes it directly for
// logging/plotting. Deliberately reads the gimbal angles from `state`,
// not from the live tvc_ member -- see tvc_'s declaration comment.
Eigen::Vector3d RocketKinematics::computeNetForce(const RocketState& state, double thrust) const {
    const double g0 = 9.80665;
    const double v = state.velocity.norm();
    const double v2 = v * v;

    Eigen::Vector3d thrust_body = -thrust * ThrustVectorControl::nozzleDirectionFromAngles(
        state.gimbal_pitch_rad, state.gimbal_yaw_rad);

    double altitude = std::max(0.0, state.position(2));
    double rho = aero_.getDensity(altitude);

    FlightConditions fc = buildFlightConditions(state);
    double drag_coeff = aero_.computeCoefficients(fc).Cd;
    double drag_area = params_.reference_area;

    Eigen::Vector3d drag_neg;
    if (v > 1e-6) {
        drag_neg = -state.velocity.normalized() * drag_coeff * drag_area * 0.5 * rho * v2;
    } else {
        drag_neg = Eigen::Vector3d::Zero();
    }

    Eigen::Matrix3d R = rocketToNedFrame(state);
    Eigen::Vector3d thrust_ned = R * thrust_body;
    Eigen::Vector3d drag_ned   = R * drag_neg;

    Eigen::Vector3d gravity(0, 0, -g0 * state.mass);

    return thrust_ned + drag_ned + gravity;
}

Eigen::Vector3d RocketKinematics::computeAcceleration(const RocketState& state, double thrust) const {
    return computeNetForce(state, thrust) / state.mass;
}

RocketState RocketKinematics::step(const RocketState& state, double thrust,
                                   const Eigen::Vector3d& tvc_target_dir) {
    // The live actuator always starts this call sitting at `state`'s own
    // gimbal angles (see tvc_'s declaration comment) -- command the new
    // target, then use THIS step's force with the actuator still at its
    // pre-command position (it hasn't physically moved yet), matching a
    // real servo's lag.
    tvc_.commandForceDirection(tvc_target_dir);

    RocketState next = state;

    Eigen::Vector3d accel = computeAcceleration(state, thrust);

    next.velocity = state.velocity + accel * config_.dt;
    next.position = state.position + next.velocity * config_.dt;

    updatePitchDynamics(next, state);
    updateYawDynamics(next, state);

    // Roll still has no torque model (see README "Staging Notes" / 6DOF
    // roadmap) -- just keep the angle bounded.
    next.orientation(0) = fmod(state.orientation(0), 2*M_PI);

    // Now advance the actuator toward its (possibly just-changed) target,
    // and bake the result into next -- this is what computeNetForce will
    // read on the FOLLOWING call.
    tvc_.step(config_.dt);
    next.gimbal_pitch_rad = tvc_.currentGimbalPitchRad();
    next.gimbal_yaw_rad = tvc_.currentGimbalYawRad();

    return next;
}

std::vector<RocketState> RocketKinematics::simulate(double time, const FlightData& flight_data,
                                                    const std::vector<Eigen::Vector3d>& tvc_targets,
                                                    Navigation* navigation) {
    int n_steps = static_cast<int>(time / config_.dt);
    if (n_steps >= (int)flight_data.time.size()) {
        n_steps = flight_data.time.size() - 1;
    }
    std::vector<RocketState> states(n_steps + 1);

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
        double thrust = flight_data.thrust[i];
        double mass_kg = gramsToKg(flight_data.mass[i]);

        Eigen::Vector3d tvc_target;
        if (navigation != nullptr) {
            // Guidance reads the state THIS step starts from -- same
            // "read the truth, no lookahead" rule everything else here
            // follows. angular_vel(0)==0 for i==0, so using states[i]
            // as its own "previous" state there just gives Gyro a
            // harmless zero angular-acceleration reading at t=0.
            nav_gps.update(states[i]);
            const RocketState& prev = (i > 0) ? states[i - 1] : states[i];
            nav_gyro.update(states[i], prev, config_.dt);
            tvc_target = navigation->computeTvcTarget(nav_gps, nav_gyro);
        } else {
            tvc_target = (i < (int)tvc_targets.size()) ? tvc_targets[i] : no_deflection;
        }

        RocketState temp = states[i];
        temp.mass = mass_kg;
        states[i + 1] = step(temp, thrust, tvc_target);

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

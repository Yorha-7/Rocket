#include <Eigen/Dense>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ============================================================
   rocket_kinematics.c - Core kinematics computation
   ============================================================
   Functions: compute_acceleration, rocket_to_ned_frame,
              step_simulation, simulate_full
   ============================================================ */

#include <stdio.h>
#include "rocket_kinematics.h"

/* Physical constants */
#define G0        9.80665       /* gravity at sea level (m/s^2) */
#define RHO0      1.225         /* sea level air density (kg/m^3) */
#define H_SCALE   8500.0        /* density scale height (m) */

/* ============================================================
   rocket_to_ned_frame - Transform body-frame forces to NED
   ============================================================ */
void rocket_to_ned_frame(const RocketState *state, Eigen::Matrix3d *R) {
    double roll  = state->orientation(0);
    double pitch = state->orientation(1);
    double yaw   = state->orientation(2);

    double cr = cos(roll),  sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw),   sy = sin(yaw);

    /* R = Rz(yaw) * Ry(pitch) * Rx(roll) */
    (*R)(0,0) = cp*cy;
    (*R)(0,1) = cp*sy*sr - sp*cr;
    (*R)(0,2) = cp*sy*cr + sp*sr;
    (*R)(1,0) = sy;
    (*R)(1,1) = cy*sr;
    (*R)(1,2) = -cy*sr*sp + cr*sy;
    (*R)(2,0) = -sp;
    (*R)(2,1) = cp*sr;
    (*R)(2,2) = cp*cr;
}

/* ============================================================
   compute_acceleration - Calculate mass-specific acceleration
   ============================================================ */
void compute_acceleration(const RocketState *state,
                          double thrust, double drag_coeff, double normal_coeff,
                          Eigen::Vector3d *accel) {
    double mass_kg = state->mass;
    double v = state->velocity.norm();
    double v2 = v * v;

    /* Air density with altitude */
    double altitude = state->position(2);
    if (altitude < 0) altitude = 0;  /* clamp */
    double rho = RHO0 * exp(-altitude / H_SCALE);

    /* Thrust in body frame: (0,0,thrust) */
    double thrust_body[3] = {0.0, 0.0, thrust};

    /* Drag force: opposes velocity, Fd = 0.5 * rho * v^2 * Cd * A */
    double drag_area = 0.005;  /* reference area m^2 (model rocket) */
    double drag_mag = 0.5 * rho * v2 * drag_area * drag_coeff;
    double drag_xy[3];

    if (v > 1e-6) {
        /* Drag opposes velocity vector */
        drag_xy[0] = -state->velocity(0) / v * drag_mag;
        drag_xy[1] = -state->velocity(1) / v * drag_mag;
        drag_xy[2] = -state->velocity(2) / v * drag_mag;
    } else {
        drag_xy[0] = drag_xy[1] = drag_xy[2] = 0.0;
    }

    /* Normal force: perpendicular to velocity and rocket axis */
    /* Simplified: cross product of velocity with up vector */
    double up[3] = {0.0, 0.0, 1.0};  /* up in NED */
    double vel_norm[3] = {
        state->velocity(0) / (v + 1e-12),
        state->velocity(1) / (v + 1e-12),
        state->velocity(2) / (v + 1e-12)
    };

    /* normal_dir = vel_norm x up */
    double normal_dir[3];
    normal_dir[0] = vel_norm[1]*up[2] - vel_norm[2]*up[1];
    normal_dir[1] = vel_norm[2]*up[0] - vel_norm[0]*up[2];
    normal_dir[2] = vel_norm[0]*up[1] - vel_norm[1]*up[0];
    double norm_len = sqrt(normal_dir[0]*normal_dir[0] +
                         normal_dir[1]*normal_dir[1] +
                         normal_dir[2]*normal_dir[2]);
    if (norm_len > 1e-12) {
        normal_dir[0] /= norm_len;
        normal_dir[1] /= norm_len;
        normal_dir[2] /= norm_len;
    } else {
        normal_dir[0] = 1.0; normal_dir[1] = 0.0; normal_dir[2] = 0.0;
    }

    double normal_force[3];
    normal_force[0] = normal_dir[0] * normal_coeff * drag_area * 0.5 * rho * v2;
    normal_force[1] = normal_dir[1] * normal_coeff * drag_area * 0.5 * rho * v2;
    normal_force[2] = normal_dir[2] * normal_coeff * drag_area * 0.5 * rho * v2;

    /* Transform forces to NED frame using rotation matrix */
    Eigen::Matrixd R;
    rocket_to_ned_frame(state, &R);

    /* Thrust in NED */
    double thrust_ned[3] = {
        R(0,0)*thrust_body[0] + R(0,1)*thrust_body[1] + R(0,2)*thrust_body[2],
        R(1,0)*thrust_body[0] + R(1,1)*thrust_body[1] + R(1,2)*thrust_body[2],
        R(2,0)*thrust_body[0] + R(2,1)*thrust_body[1] + R(2,2)*thrust_body[2]
    };

    /* Drag in NED */
    double drag_ned[3] = {drag_xy[0], drag_xy[1], drag_xy[2]};

    /* Normal in NED */
    double normal_ned[3] = {normal_force[0], normal_force[1], normal_force[2]};

    /* Total force in NED */
    double total_force[3] = {
        thrust_ned[0] + drag_ned[0] + normal_ned[0],
        thrust_ned[1] + drag_ned[1] + normal_ned[1],
        thrust_ned[2] + drag_ned[2] + normal_ned[2]
    };

    /* Gravity in NED: (0, 0, -g0 * mass) */
    double gravity[3] = {0.0, 0.0, -G0 * mass_kg};

    /* Mass-specific acceleration = (F_total + gravity) / mass */
    accel->x() = (total_force[0] + gravity[0]) / mass_kg;
    accel->y() = (total_force[1] + gravity[1]) / mass_kg;
    accel->z() = (total_force[2] + gravity[2]) / mass_kg;
}

/* ============================================================
   step_simulation - Advance one integration step
   ============================================================ */
void step_simulation(const RocketState *state, double thrust,
                    double drag_coeff, double normal_coeff,
                    double dt, RocketState *next) {
    Eigen::Vector3d accel;
    compute_acceleration(state, thrust, drag_coeff, normal_coeff, &accel);

    /* Update velocity (Euler) */
    next->velocity = state->velocity;
    next->velocity.x() += accel.x() * dt;
    next->velocity.y() += accel.y() * dt;
    next->velocity.z() += accel.z() * dt;

    /* Update position */
    next->position = state->position;
    next->position.x() += next->velocity.x() * dt;
    next->position.y() += next->velocity.y() * dt;
    next->position.z() += next->velocity.z() * dt;

    /* Update orientation - simple pitch rate from vertical acceleration */
    next->orientation = state->orientation;
    if (state->velocity.squaredNorm() > 1e-12) {
        double pitch_rate = accel.y() / (state->velocity.norm() + 1e-6) * dt;
        next->orientation.y() += pitch_rate;
    }

    /* Keep angles bounded */
    next->orientation.x() = fmod(next->orientation.x(), 2*M_PI);
    next->orientation.z() = fmod(next->orientation.z(), 2*M_PI);
}

/* ============================================================
   simulate_full - Run complete trajectory simulation
   ============================================================ */
int simulate_full(const RocketParams *params,
                  const SimulationConfig *config,
                  const double *thrust_data, int thrust_len,
                  const double *drag_coeffs, const double *normal_coeffs,
                  RocketState *states, int *n_out) {
    int n_steps = (int)(config->sim_duration / config->dt);
    if (n_steps < 1) n_steps = 1;
    if (n_steps > thrust_len) n_steps = thrust_len;

    /* Initial state */
    states[0].position.setZero();
    states[0].position(2) = config->launch_height;  /* z = height above ground */
    states[0].velocity.setZero();
    states[0].orientation.setZero();
    states[0].orientation(1) = config->init_tilt * M_PI / 180.0;  /* pitch from vertical */
    states[0].mass = params->cg_location / 1000.0;  /* grams to kg */

    /* Current mass (decreases during burn) */
    double mass = states[0].mass;
    double mass_burn_rate = mass / params->thrust_duration;  /* kg/s */

    /* Track ground contact */
    int ground_contact_step = -1;

    for (int i = 0; i < n_steps; ++i) {
        double t = i * config->dt;

        /* Interpolate thrust */
        double thrust = 0.0;
        if (t < params->thrust_duration && i < thrust_len - 1) {
            int idx = (int)(t / config->dt);
            if (idx >= thrust_len - 1) idx = thrust_len - 2;
            double t_frac = (t - idx * config->dt) / config->dt;
            thrust = thrust_data[idx] * (1.0 - t_frac) + thrust_data[idx+1] * t_frac;

            /* Mass decreases during burn */
            mass = states[0].mass - mass_burn_rate * t;
        }

        /* Drag / normal coeff lookup */
        double drag_c  = (drag_coeffs)  ? drag_coeffs[i % thrust_len] : 0.0;
        double normal_c = (normal_coeffs) ? normal_coeffs[i % thrust_len] : 0.0;

        /* Advance one step */
        step_simulation(&states[i], thrust, drag_c, normal_c,
                        config->dt, &states[i+1]);

        /* *** GROUND TERMINATION CHECK ***
           If z-position goes below ground (z < 0), stop simulation.
           Ground is at z = 0 when launch_height = 0. */
        if (states[i+1].position(2) < 0.0 && ground_contact_step < 0) {
            ground_contact_step = i + 1;

            /* Interpolate contact time */
            double z_above = states[i].position(2);
            double z_below = states[i+1].position(2);
            if (z_above > 0 && z_below < 0) {
                /* Linear interpolation for exact contact */
                /* We stop here; remaining states will be filled below */
            }
        }
    }

    *n_out = n_steps + 1;

    /* Fill any remaining states at ground level if contact occurred */
    if (ground_contact_step > 0 && ground_contact_step < *n_out) {
        for (int j = ground_contact_step; j < *n_out; ++j) {
            states[j] = states[ground_contact_step - 1];
            states[j].velocity.setZero();
            states[j].orientation.setZero();
        }
    }

    return 0;
}
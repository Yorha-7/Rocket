#include "rocket_kinematics.hpp"
#include "ork_loader.hpp"
#include "aerodynamics.hpp"
#include "mass_properties_model.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <cstdlib>
#include <unistd.h>

int main() {
    // ============================================================
    // Load the rocket's geometry, launch conditions, and full flight
    // history (thrust/mass/drag/CP/CG/inertia over time) straight from
    // its OpenRocket design file. No parameters are hand-copied here.
    // ============================================================
    const std::string ork_path = "/media/jayesh/Acer/Users/scien/Rocket/artifacts/rocket.ork";

    SimulationConfig config;
    config.dt = 0.01;            // our integrator's step size, not part of the rocket's design
    config.sim_duration = 120.0; // upper bound; the sim stops at ground contact regardless

    std::cout << "Loading rocket design and flight data from " << ork_path << "...\n";
    OrkRocket rocket = loadOrkRocket(ork_path, config.dt);

    config.launch_height = rocket.launch_conditions.launch_height;
    config.init_tilt = rocket.launch_conditions.init_tilt;

    // Every simulation embedded in this .ork launches from a dead-vertical
    // rod (launchrodangle = 0), so with no perturbation the pitch model
    // never has anything to restore from. Override with a small nonzero
    // tilt so the pitch dynamics (gravity/aero/damping torque) actually
    // show something, instead of leaving it real but silent.
    const double INIT_TILT_OVERRIDE_DEG = 3.0;
    config.init_tilt = INIT_TILT_OVERRIDE_DEG;

    const RocketParams& params = rocket.params;
    const FlightData& flight_data = rocket.flight_data;

    std::cout << "Nose: " << params.nose_length << " m, shape code " << params.nose_shape << "\n";
    std::cout << "Body: " << params.body_length << " m long, " << params.body_diameter << " m diameter\n";
    std::cout << "Fins: " << params.fin_count << " x (root " << params.fin_root_chord
              << " m, tip " << params.fin_tip_chord << " m, span " << params.fin_span << " m)\n";
    std::cout << "Reference area: " << params.reference_area << " m^2\n";
    std::cout << "Loaded " << flight_data.time.size() << " flight-data points (thrust/mass)\n";
    std::cout << "Total liftoff mass (from .ork motor data): " << flight_data.mass.front() << " g\n";

    // ============================================================
    // Sanity-check our own computed coefficients/mass-properties (not
    // read from the .ork -- these come from AerodynamicsModel and
    // VehicleMassModel, built from the vehicle's own geometry).
    // ============================================================
    AerodynamicsModel aero(params);
    VehicleMassModel mass_model(rocket.mass_components, params.body_diameter, params.body_length);

    double cp_cm = aero.computeCenterOfPressure();
    MassProperties dry_mp = mass_model.computeAt(mass_model.dryMassKg());
    std::cout << "Computed dry structure mass: " << mass_model.dryMassKg() * 1000.0 << " g, "
              << "dry CG: " << mass_model.dryCgCm() << " cm from nose tip\n";
    std::cout << "Computed CP (Barrowman): " << cp_cm << " cm from nose tip\n";
    std::cout << "Computed dry I_yy: " << dry_mp.I_yy << " kg*m^2\n";

    FlightConditions sample_fc{};
    sample_fc.altitude = 100.0;
    sample_fc.velocity = 50.0;
    sample_fc.mach = sample_fc.velocity / aero.getSpeedOfSound(sample_fc.altitude);
    sample_fc.alpha = 0.05;  // ~3 deg, representative mid-flight angle of attack
    AerodynamicCoefficients sample_coeffs = aero.computeCoefficients(sample_fc);
    std::cout << "Computed Cd @ 50 m/s, 100 m altitude: " << sample_coeffs.Cd
              << " (Cn_alpha=" << sample_coeffs.Cn_alpha << "/rad)\n";

    // ============================================================
    // Run the simulation
    // ============================================================
    RocketKinematics sim(params, config, rocket.mass_components);

    std::cout << "\nRocket Simulation (3DOF translation + pitch) with Ground Termination\n";
    std::cout << "==================================================\n";
    std::cout << "Config: launch_height=" << config.launch_height
              << "m, init_tilt=" << config.init_tilt
              << "deg, dt=" << config.dt << "s\n";

    auto states = sim.simulate(config.sim_duration, flight_data);

    // ============================================================
    // Extract time series for output and plotting
    // ============================================================
    std::vector<double> time_vec, height_vec, velocity_vec, pitch_vec;
    time_vec.reserve(states.size());
    height_vec.reserve(states.size());
    velocity_vec.reserve(states.size());
    pitch_vec.reserve(states.size());

    for (size_t i = 0; i < states.size(); ++i) {
        time_vec.push_back(i * config.dt);
        height_vec.push_back(states[i].position(2));
        velocity_vec.push_back(states[i].velocity.norm());
        pitch_vec.push_back(states[i].orientation(1) * 180.0 / M_PI);
    }

    // Angular velocity (pitch rate) and angular acceleration
    std::vector<double> ang_vel_vec(states.size(), 0.0);
    std::vector<double> ang_accel_vec(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
        ang_vel_vec[i] = states[i].angular_vel(1) * 180.0 / M_PI; // deg/s
    }
    for (size_t i = 1; i < states.size(); ++i) {
        ang_accel_vec[i] = (ang_vel_vec[i] - ang_vel_vec[i-1]) / config.dt; // deg/s^2
    }
    ang_accel_vec[0] = ang_accel_vec[1];

    // Find apogee
    double t_apogee = 0.0, h_apogee = 0.0;
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i].position(2) > h_apogee) {
            h_apogee = states[i].position(2);
            t_apogee = i * config.dt;
        }
    }

    // Net world-frame force and the pitch-torque breakdown at each logged
    // state -- not part of the integration itself, just recomputed from
    // the same physics for the Forces/Torque Analysis plots.
    std::vector<double> fx_vec(states.size(), 0.0), fy_vec(states.size(), 0.0), fz_vec(states.size(), 0.0);
    std::vector<double> torque_gravity_vec(states.size(), 0.0);
    std::vector<double> torque_aero_vec(states.size(), 0.0);
    std::vector<double> torque_damping_vec(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
        double thrust_i = (i < flight_data.thrust.size()) ? flight_data.thrust[i] : 0.0;
        Eigen::Vector3d force = sim.computeNetForce(states[i], thrust_i);
        fx_vec[i] = force(0);
        fy_vec[i] = force(1);
        fz_vec[i] = force(2);

        PitchTorques torques = sim.computePitchTorques(states[i]);
        torque_gravity_vec[i] = torques.gravity;
        torque_aero_vec[i] = torques.aerodynamic;
        torque_damping_vec[i] = torques.damping;
    }

    // ============================================================
    // Save CSV output
    // ============================================================
    std::ofstream csv("rocket_trajectory.csv");
    csv << "time,height,velocity,pitch,ang_vel,ang_accel,"
        << "fx,fy,fz,torque_gravity,torque_aero,torque_damping\n";
    for (size_t i = 0; i < states.size(); ++i) {
        csv << std::fixed << std::setprecision(6);
        csv << time_vec[i] << "," << height_vec[i] << "," << velocity_vec[i] << ","
            << pitch_vec[i] << "," << ang_vel_vec[i] << "," << ang_accel_vec[i] << ","
            << fx_vec[i] << "," << fy_vec[i] << "," << fz_vec[i] << ","
            << torque_gravity_vec[i] << "," << torque_aero_vec[i] << "," << torque_damping_vec[i] << "\n";
    }
    csv.close();

    // ============================================================
    // Generate PNG plot via Python/matplotlib
    // ============================================================
    std::cout << "\nGenerating plot via Python/matplotlib...\n";
    const char* project_root = "/media/jayesh/Acer/Users/scien/Rocket/rocket_cpp";
    if (chdir(project_root) == 0) {
        int result = system("python3 scripts/plot_trajectory.py rocket_trajectory.csv rocket_trajectory.png");
        if (result != 0) {
            std::cerr << "Warning: Python plot generation failed (matplotlib not installed?). Continuing...\n";
        } else {
            std::cout << "Plot saved to rocket_trajectory.png\n";
        }
    } else {
        std::cerr << "Warning: Could not change to project root directory. Skipping plot.\n";
    }

    // ============================================================
    // Console summary
    // ============================================================
    std::cout << "\nApogee: " << h_apogee << " m at t=" << t_apogee << " s\n";
    std::cout << "Simulation complete.\n";
    std::cout << "Results: rocket_trajectory.csv, rocket_trajectory.png\n";
    std::cout << "Ground termination: stops when z < 0\n";

    return 0;
}

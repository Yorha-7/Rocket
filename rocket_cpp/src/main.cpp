#include "sim/rocket_kinematics.hpp"
#include "gnc/navigation.hpp"
#include "ork/ork_loader.hpp"
#include "sim/aerodynamics.hpp"
#include "sim/mass_properties_model.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cstdlib>
#include <unistd.h>
#include <climits>

// ##### findProjectRoot() #####
// Goal: work out where THIS repo actually lives on disk, instead of
// hardcoding one machine's own absolute path -- reads /proc/self/exe
// (the running binary's own real, resolved location, regardless of how
// it was invoked or what the current directory happens to be) and walks
// up from there. The executable always lives at <repo_root>/rocket_cpp/
// build/rocket_cpp, so two levels up from its own directory is the repo
// root.
std::string findProjectRoot() {
    char path[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", path, sizeof(path) - 1);
    if (len == -1) return ".";  // fallback: trust the caller's own working directory
    path[len] = '\0';

    std::string exe_dir(path);
    size_t last_slash = exe_dir.find_last_of('/');
    exe_dir = (last_slash == std::string::npos) ? "." : exe_dir.substr(0, last_slash);

    return exe_dir + "/..";  // build/ -> rocket_cpp/
}

// ##### loadTvcTestSequence() #####
// Goal: read data/tvc_test_sequence.csv (time-segment table: t_start_s,
// t_end_s, target_x,target_y,target_z, note) and expand it into one
// target-direction vector per simulation step, so
// RocketKinematics::simulate() can command ThrustVectorControl the same
// way every step without knowing about time segments itself. Steps past
// the last segment's t_end_s get no deflection.
std::vector<Eigen::Vector3d> loadTvcTestSequence(const std::string& csv_path, double dt, int n_steps) {
    struct Segment { double t_start, t_end; Eigen::Vector3d target; };
    std::vector<Segment> segments;

    std::ifstream file(csv_path);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;
        while (std::getline(ss, field, ',')) fields.push_back(field);
        if (fields.size() < 5) continue;
        segments.push_back({std::stod(fields[0]), std::stod(fields[1]),
                             Eigen::Vector3d(std::stod(fields[2]), std::stod(fields[3]), std::stod(fields[4]))});
    }

    std::vector<Eigen::Vector3d> targets(n_steps, Eigen::Vector3d(0, 0, 1));
    for (int i = 0; i < n_steps; ++i) {
        double t = i * dt;
        for (const auto& seg : segments) {
            if (t >= seg.t_start && t < seg.t_end) {
                targets[i] = seg.target;
                break;
            }
        }
    }
    return targets;
}

int main() {
    // ##### Load the rocket #####
    // Goal: pull geometry, launch conditions, and the full flight
    // history (thrust/mass over time) straight from the OpenRocket
    // design file -- no parameters hand-copied here. Path is built from
    // the repo root (see findProjectRoot()), not one machine's hardcoded
    // absolute path.
    const std::string project_root = findProjectRoot();
    const std::string ork_path = project_root + "/../artifacts/rocket.ork";

    SimulationConfig config;
    config.dt = 0.001;            // our integrator's step size, not part of the rocket's design
    // Upper bound only -- ground contact ends the sim earlier in the
    // normal case. 300s (was 120s) gives room for a wide/high off-axis
    // target's trajectory to actually finish (see README Staging Notes).
    config.sim_duration = 300.0;

    std::cout << "Loading rocket design and flight data from " << ork_path << "...\n";
    OrkRocket rocket = loadOrkRocket(ork_path, config.dt);

    config.launch_height = rocket.launch_conditions.launch_height;
    config.init_tilt = rocket.launch_conditions.init_tilt;

    // Goal: every simulation embedded in this .ork launches from a
    // dead-vertical rod, so with zero perturbation the pitch model would
    // never have anything to restore from. Override with a small nonzero
    // tilt so pitch dynamics actually show something real, not silent.
    const double INIT_TILT_OVERRIDE_DEG = 0.0;
    config.init_tilt = INIT_TILT_OVERRIDE_DEG;

    // Goal: same idea for yaw -- the .ork has no yaw/azimuth concept at
    // all, so this is a fixed launch-azimuth deviation, not anything
    // read from the design file.
    const double INIT_YAW_OVERRIDE_DEG = 10.0;
    config.init_yaw = INIT_YAW_OVERRIDE_DEG;

    const RocketParams& params = rocket.params;
    const FlightData& flight_data = rocket.flight_data;

    std::cout << "Nose: " << params.nose_length << " m, shape code " << params.nose_shape << "\n";
    std::cout << "Body: " << params.body_length << " m long, " << params.body_diameter << " m diameter\n";
    std::cout << "Fins: " << params.fin_count << " x (root " << params.fin_root_chord
              << " m, tip " << params.fin_tip_chord << " m, span " << params.fin_span << " m)\n";
    std::cout << "Reference area: " << params.reference_area << " m^2\n";
    std::cout << "Loaded " << flight_data.time.size() << " flight-data points (thrust/mass)\n";
    std::cout << "Total liftoff mass (from .ork motor data): " << flight_data.mass.front() << " g\n";

    // ##### Sanity-check our own computed physics #####
    // Goal: print what AerodynamicsModel/VehicleMassModel work out from
    // the vehicle's own geometry (NOT read from the .ork's own solved
    // simulation) so a bad geometry read shows up here, before a full
    // flight is even run.
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

    // ##### Run the simulation #####
    RocketKinematics sim(params, config, rocket.mass_components);

    std::cout << "\nRocket Simulation (3DOF translation + pitch/yaw dynamics + TVC/Navigation) "
                 "with Ground Termination\n";
    std::cout << "==================================================\n";
    std::cout << "Config: launch_height=" << config.launch_height
              << "m, init_tilt=" << config.init_tilt
              << "deg, dt=" << config.dt << "s\n";

    // Goal: fix the guidance target for this run -- hardcoded for now,
    // no mission-planning input yet (staging-area scope).
    const Eigen::Vector3d NAV_TARGET(0.0, 350.0, 1500.0);
    Navigation navigation(NAV_TARGET, config.dt);
    std::cout << "Navigation target: (" << NAV_TARGET.x() << ", " << NAV_TARGET.y()
              << ", " << NAV_TARGET.z() << ") m -- guidance law: point the nose at it, TVC does the rest\n";

    auto states = sim.simulate(config.sim_duration, flight_data, {}, &navigation);

    // ##### Extract time series for output and plotting #####
    // Goal: unpack the raw RocketState history into the flat per-column
    // arrays the CSV writer and matplotlib scripts actually want.
    std::vector<double> time_vec, x_vec, y_vec, height_vec, velocity_vec, pitch_vec, yaw_vec;
    std::vector<double> gimbal_pitch_vec, gimbal_yaw_vec;
    time_vec.reserve(states.size());
    x_vec.reserve(states.size());
    y_vec.reserve(states.size());
    height_vec.reserve(states.size());
    velocity_vec.reserve(states.size());
    pitch_vec.reserve(states.size());
    yaw_vec.reserve(states.size());
    gimbal_pitch_vec.reserve(states.size());
    gimbal_yaw_vec.reserve(states.size());

    for (size_t i = 0; i < states.size(); ++i) {
        time_vec.push_back(i * config.dt);
        x_vec.push_back(states[i].position(0));
        y_vec.push_back(states[i].position(1));
        height_vec.push_back(states[i].position(2));
        velocity_vec.push_back(states[i].velocity.norm());
        pitch_vec.push_back(states[i].orientation(1) * 180.0 / M_PI);
        yaw_vec.push_back(states[i].orientation(2) * 180.0 / M_PI);
        gimbal_pitch_vec.push_back(states[i].gimbal_pitch_rad * 180.0 / M_PI);
        gimbal_yaw_vec.push_back(states[i].gimbal_yaw_rad * 180.0 / M_PI);
    }

    // Goal: angular velocity/acceleration aren't stored directly --
    // derive rate from the state history, then acceleration by
    // differencing that.
    std::vector<double> ang_vel_vec(states.size(), 0.0);
    std::vector<double> ang_accel_vec(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
        ang_vel_vec[i] = states[i].angular_vel(1) * 180.0 / M_PI; // deg/s
    }
    for (size_t i = 1; i < states.size(); ++i) {
        ang_accel_vec[i] = (ang_vel_vec[i] - ang_vel_vec[i-1]) / config.dt; // deg/s^2
    }
    ang_accel_vec[0] = ang_accel_vec[1];

    // Goal: find the highest point the flight actually reached, and when.
    double t_apogee = 0.0, h_apogee = 0.0;
    for (size_t i = 0; i < states.size(); ++i) {
        if (states[i].position(2) > h_apogee) {
            h_apogee = states[i].position(2);
            t_apogee = i * config.dt;
        }
    }

    // Goal: recompute the same force/torque breakdown step() used
    // internally, purely for the CSV's Forces/Torque Analysis columns --
    // not part of the integration itself.
    std::vector<double> fx_vec(states.size(), 0.0), fy_vec(states.size(), 0.0), fz_vec(states.size(), 0.0);
    std::vector<double> thrust_vec(states.size(), 0.0);
    std::vector<double> torque_gravity_vec(states.size(), 0.0);
    std::vector<double> torque_aero_vec(states.size(), 0.0);
    std::vector<double> torque_damping_vec(states.size(), 0.0);
    for (size_t i = 0; i < states.size(); ++i) {
        double thrust_i = (i < flight_data.thrust.size()) ? flight_data.thrust[i] : 0.0;
        thrust_vec[i] = thrust_i;
        Eigen::Vector3d force = sim.computeNetForce(states[i], thrust_i);
        fx_vec[i] = force(0);
        fy_vec[i] = force(1);
        fz_vec[i] = force(2);

        PitchTorques torques = sim.computePitchTorques(states[i]);
        torque_gravity_vec[i] = torques.gravity;
        torque_aero_vec[i] = torques.aerodynamic;
        torque_damping_vec[i] = torques.damping;
    }

    // ##### Save CSV output #####
    // Goal: land the CSV in the project root every time, regardless of
    // which directory the binary was launched from -- otherwise running
    // from build/ used to leave a stale CSV behind for the plot script
    // to read instead of this run's fresh one. Reuses the same
    // project_root computed above, not a second hardcoded path.
    bool in_project_root = (chdir(project_root.c_str()) == 0);
    if (!in_project_root) {
        std::cerr << "Warning: could not chdir to project root; writing CSV to current directory.\n";
    }
    std::ofstream csv("rocket_trajectory.csv");
    csv << "time,x,y,height,velocity,pitch,yaw,ang_vel,ang_accel,"
        << "fx,fy,fz,thrust,torque_gravity,torque_aero,torque_damping,gimbal_pitch_deg,gimbal_yaw_deg,"
        << "target_x,target_y,target_z\n";
    for (size_t i = 0; i < states.size(); ++i) {
        csv << std::fixed << std::setprecision(6);
        csv << time_vec[i] << "," << x_vec[i] << "," << y_vec[i] << "," << height_vec[i] << ","
            << velocity_vec[i] << ","
            << pitch_vec[i] << "," << yaw_vec[i] << "," << ang_vel_vec[i] << "," << ang_accel_vec[i] << ","
            << fx_vec[i] << "," << fy_vec[i] << "," << fz_vec[i] << "," << thrust_vec[i] << ","
            << torque_gravity_vec[i] << "," << torque_aero_vec[i] << "," << torque_damping_vec[i] << ","
            << gimbal_pitch_vec[i] << "," << gimbal_yaw_vec[i] << ","
            << NAV_TARGET.x() << "," << NAV_TARGET.y() << "," << NAV_TARGET.z() << "\n";
    }
    csv.close();

    // ##### Generate PNG plot via Python/matplotlib #####
    std::cout << "\nGenerating plots via Python/matplotlib...\n";
    if (in_project_root) {
        int result = system("python3 scripts/plot_trajectory.py rocket_trajectory.csv rocket_analysis.png");
        if (result != 0) {
            std::cerr << "Warning: Python plot generation failed (matplotlib not installed?). Continuing...\n";
        } else {
            std::cout << "Plots saved to rocket_analysis.png, rocket_trajectory.png\n";
        }
    } else {
        std::cerr << "Warning: Could not change to project root directory. Skipping plot.\n";
    }

    // ##### Console summary #####
    std::cout << "\nApogee: " << h_apogee << " m at t=" << t_apogee << " s\n";
    std::cout << "Simulation complete.\n";
    std::cout << "Results: rocket_trajectory.csv, rocket_analysis.png, rocket_trajectory.png\n";
    std::cout << "Ground termination: stops when z < 0\n";

    return 0;
}

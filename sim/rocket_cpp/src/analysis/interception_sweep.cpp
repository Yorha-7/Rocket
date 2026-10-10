#include "project_settings.hpp"
#include "cli_args.hpp"
#include "analysis/interception_sweep.hpp"
#include "gnc/navigation.hpp"
#include "ork/ork_loader.hpp"
#include "sim/rocket_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace interception;

struct ClosestApproach {
    double distance_m = std::numeric_limits<double>::infinity();
    double time_s = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

struct SweepResult {
    Eigen::Vector3d target = Eigen::Vector3d::Zero();
    ClosestApproach closest;
    double target_radius_m = 0.0;
    double apogee_m = 0.0;
    double impact_time_s = 0.0;
    double max_gimbal_pitch_deg = 0.0;
    double max_gimbal_yaw_deg = 0.0;
    size_t waypoint_index = 0;
    size_t waypoint_count = 0;
    bool final_waypoint_reached = false;
    bool ground_terminated = false;
    bool terminal_guidance = false;
};

double parseDouble(const std::string& value, const char* option) {
    try {
        size_t consumed = 0;
        double parsed = std::stod(value, &consumed);
        if (consumed != value.size()) throw std::invalid_argument("trailing characters");
        return parsed;
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string("invalid value for ") + option + ": " + value);
    }
}

int parseInt(const std::string& value, const char* option) {
    try {
        size_t consumed = 0;
        int parsed = std::stoi(value, &consumed);
        if (consumed != value.size()) throw std::invalid_argument("trailing characters");
        return parsed;
    } catch (const std::exception&) {
        throw std::invalid_argument(std::string("invalid value for ") + option + ": " + value);
    }
}

SweepOptions parseOptions(int argc, char** argv, const std::string& project_root) {
    SweepOptions options;
    options.output_path = project_root + project_paths::SWEEP_BASELINE_CSV;

    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        auto requireValue = [&](const char* option) -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument(std::string("missing value for ") + option);
            }
            return argv[++i];
        };

        if (flag == "--radius") {
            options.radius_m = parseDouble(requireValue("--radius"), "--radius");
        } else if (flag == "--shells") {
            options.shells = parseInt(requireValue("--shells"), "--shells");
        } else if (flag == "--directions") {
            options.directions = parseInt(requireValue("--directions"), "--directions");
        } else if (flag == "--init-tilt") {
            options.init_tilt_deg = parseDouble(requireValue("--init-tilt"), "--init-tilt");
        } else if (flag == "--init-yaw") {
            options.init_yaw_deg = parseDouble(requireValue("--init-yaw"), "--init-yaw");
        } else if (flag == "--output") {
            options.output_path = requireValue("--output");
            options.output_explicit = true;
        } else if (flag == "--terminal") {
            options.terminal = true;
        } else if (flag == "--baseline") {
            options.terminal = false;
        } else if (flag == "--full") {
            options.preview = false;
        } else if (flag == "--preview") {
            options.preview = true;
        } else if (flag == "--help" || flag == "-h") {
            std::cout
                << "Usage: interception_sweep [options]\n"
                << "  --radius R       sphere radius in metres (default " << DEFAULT_RADIUS_M << ")\n"
                << "  --shells N       radial shells (default " << DEFAULT_SHELLS << ")\n"
                << "  --directions N   upper-hemisphere directions per shell (default " << DEFAULT_DIRECTIONS << ")\n"
                << "  --preview        dt=" << simulation_settings::PREVIEW_DT_S << ", "
                << simulation_settings::PREVIEW_DURATION_S << " s cap (default)\n"
                << "  --full           dt=" << simulation_settings::FULL_DT_S << ", "
                << simulation_settings::FULL_DURATION_S << " s cap\n"
                << "  --terminal       use target-relative terminal interception guidance\n"
                << "  --baseline       use the existing waypoint/PID guidance (default)\n"
                << "  --init-tilt D    initial pitch tilt in degrees\n"
                << "  --init-yaw D     launch yaw in degrees\n"
                << "  --output FILE    result CSV path\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown option: " + flag);
        }
    }

    if (!options.output_explicit) {
        options.output_path = project_root + (options.terminal ? project_paths::SWEEP_TERMINAL_CSV
                                                       : project_paths::SWEEP_BASELINE_CSV);
    }
    if (!(options.radius_m > 0.0) || options.shells < 1 || options.directions < 4) {
        throw std::invalid_argument("radius must be positive, shells >= 1, directions >= 4");
    }
    return options;
}

std::vector<Eigen::Vector3d> buildTargets(const SweepOptions& options) {
    std::vector<Eigen::Vector3d> targets;
    // Include a near-vertical reference target. The exact sphere centre is
    // on the pad and is intentionally not used because Navigation rejects
    // targets at or below its 10 m ground-safety floor.
    targets.emplace_back(0.0, 0.0, MIN_TARGET_ALTITUDE_M + REFERENCE_TARGET_CLEARANCE_M);

    // Fibonacci directions give repeatable, approximately uniform coverage
    // of the physically valid upper hemisphere without polar clustering.
    const double golden_angle = PI * (3.0 - std::sqrt(5.0));
    for (int shell = 1; shell <= options.shells; ++shell) {
        const double radius = options.radius_m * static_cast<double>(shell) / options.shells;
        for (int i = 0; i < options.directions; ++i) {
            const double z_unit = (static_cast<double>(i) + 0.5) / options.directions;
            const double planar = std::sqrt(std::max(0.0, 1.0 - z_unit * z_unit));
            const double azimuth = golden_angle * i;
            Eigen::Vector3d target(
                radius * planar * std::cos(azimuth),
                radius * planar * std::sin(azimuth),
                radius * z_unit);
            if (target.z() > MIN_TARGET_ALTITUDE_M) targets.push_back(target);
        }
    }
    return targets;
}

ClosestApproach findClosestApproach(const std::vector<RocketState>& states,
                                    const Eigen::Vector3d& target,
                                    double dt) {
    ClosestApproach result;
    if (states.empty()) return result;

    result.position = states.front().position;
    result.distance_m = (result.position - target).norm();

    for (size_t i = 1; i < states.size(); ++i) {
        const Eigen::Vector3d segment = states[i].position - states[i - 1].position;
        const double segment_squared = segment.squaredNorm();
        double fraction = 0.0;
        if (segment_squared > MIN_SEGMENT_SQUARED_M2) {
            fraction = ((target - states[i - 1].position).dot(segment)) / segment_squared;
            fraction = std::clamp(fraction, 0.0, 1.0);
        }
        const Eigen::Vector3d candidate = states[i - 1].position + fraction * segment;
        const double distance = (candidate - target).norm();
        if (distance < result.distance_m) {
            result.distance_m = distance;
            result.position = candidate;
            result.time_s = (static_cast<double>(i - 1) + fraction) * dt;
        }
    }
    return result;
}

SweepResult runTarget(const OrkRocket& rocket, const SweepOptions& options,
                      const Eigen::Vector3d& target) {
    SimulationConfig config{};
    config.dt = options.preview ? simulation_settings::PREVIEW_DT_S : simulation_settings::FULL_DT_S;
    config.sim_duration = options.preview ? simulation_settings::PREVIEW_DURATION_S : simulation_settings::FULL_DURATION_S;
    config.launch_height = rocket.launch_conditions.launch_height;
    config.init_tilt = options.init_tilt_deg;
    config.init_yaw = options.init_yaw_deg;

    RocketKinematics simulator(rocket.params, config, rocket.mass_components);
    navigation::Navigation navigation(target, config.dt,
                                       navigation::NavigationStep::DEFAULT_KP,
                                       navigation::NavigationStep::DEFAULT_KI,
                                       navigation::NavigationStep::DEFAULT_KD,
                                       options.terminal);
    const std::vector<RocketState> states =
        simulator.simulate(config.sim_duration, rocket.flight_data, {}, &navigation);

    SweepResult result;
    result.target = target;
    result.target_radius_m = target.norm();
    result.closest = findClosestApproach(states, target, config.dt);
    result.waypoint_index = navigation.currentWaypointIndex();
    result.waypoint_count = navigation.waypointCount();
    result.final_waypoint_reached =
        result.waypoint_count > 0 && result.waypoint_index + 1 >= result.waypoint_count;
    result.ground_terminated = !states.empty() && states.back().position.z() <= 0.0;
    result.terminal_guidance = options.terminal;
    result.impact_time_s = states.empty() ? 0.0 : (states.size() - 1) * config.dt;

    for (const RocketState& state : states) {
        result.apogee_m = std::max(result.apogee_m, state.position.z());
        result.max_gimbal_pitch_deg = std::max(
            result.max_gimbal_pitch_deg, std::abs(state.gimbal_pitch_rad * 180.0 / PI));
        result.max_gimbal_yaw_deg = std::max(
            result.max_gimbal_yaw_deg, std::abs(state.gimbal_yaw_rad * 180.0 / PI));
    }
    return result;
}

void writeHeader(std::ofstream& output) {
    output << "target_x,target_y,target_z,target_radius,"
           << "miss_distance,t_closest,closest_x,closest_y,closest_z,"
           << "apogee,impact_time,max_gimbal_pitch_deg,max_gimbal_yaw_deg,"
           << "waypoint_index,waypoint_count,final_waypoint_reached,ground_terminated,"
           << "terminal_guidance\n";
}

void writeResult(std::ofstream& output, const SweepResult& result) {
    output << std::fixed << std::setprecision(simulation_settings::CSV_PRECISION)
           << result.target.x() << ',' << result.target.y() << ',' << result.target.z() << ','
           << result.target_radius_m << ',' << result.closest.distance_m << ','
           << result.closest.time_s << ',' << result.closest.position.x() << ','
           << result.closest.position.y() << ',' << result.closest.position.z() << ','
           << result.apogee_m << ',' << result.impact_time_s << ','
           << result.max_gimbal_pitch_deg << ',' << result.max_gimbal_yaw_deg << ','
           << result.waypoint_index << ',' << result.waypoint_count << ','
           << (result.final_waypoint_reached ? 1 : 0) << ','
           << (result.ground_terminated ? 1 : 0) << ','
           << (result.terminal_guidance ? 1 : 0) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string project_root = findProjectRoot();
        const SweepOptions options = parseOptions(argc, argv, project_root);
        const std::string ork_path = project_root + project_paths::ROCKET_DESIGN;
        const double dt = options.preview ? simulation_settings::PREVIEW_DT_S : simulation_settings::FULL_DT_S;

        std::cout << "Loading rocket design from " << ork_path << "...\n";
        const OrkRocket rocket = loadOrkRocket(ork_path, dt);
        const std::vector<Eigen::Vector3d> targets = buildTargets(options);

        const std::filesystem::path output_path(options.output_path);
        if (output_path.has_parent_path()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        std::ofstream output(output_path);
        if (!output) {
            throw std::runtime_error("could not open output CSV: " + output_path.string());
        }
        writeHeader(output);

        std::cout << "Running " << targets.size() << " target cases in a "
                  << options.radius_m << " m upper-hemisphere sweep ("
                  << (options.preview ? "preview" : "full") << ", "
                  << (options.terminal ? "terminal guidance" : "baseline guidance")
                  << ")...\n";

        std::vector<double> errors;
        errors.reserve(targets.size());
        double max_error = 0.0;
        size_t successes_5m = 0;
        size_t successes_10m = 0;
        size_t successes_25m = 0;

        for (size_t i = 0; i < targets.size(); ++i) {
            const SweepResult result = runTarget(rocket, options, targets[i]);
            writeResult(output, result);
            errors.push_back(result.closest.distance_m);
            max_error = std::max(max_error, result.closest.distance_m);
            if (result.closest.distance_m <= SUCCESS_NEAR_M) ++successes_5m;
            if (result.closest.distance_m <= SUCCESS_MEDIUM_M) ++successes_10m;
            if (result.closest.distance_m <= SUCCESS_FAR_M) ++successes_25m;

            if ((i + 1) % PROGRESS_INTERVAL == 0 || i + 1 == targets.size()) {
                std::cout << "  completed " << (i + 1) << '/' << targets.size() << '\n';
            }
        }
        output.close();

        std::sort(errors.begin(), errors.end());
        const double median = errors.empty() ? 0.0 : errors[errors.size() / 2];
        std::cout << std::fixed << std::setprecision(3)
                  << "\nSweep complete.\n"
                  << "Results: " << options.output_path << '\n'
                  << "Minimum miss: " << (errors.empty() ? 0.0 : errors.front()) << " m\n"
                  << "Median miss: " << median << " m\n"
                  << "Maximum miss: " << max_error << " m\n"
                  << "Within " << SUCCESS_NEAR_M << " m: " << successes_5m << '/' << targets.size() << '\n'
                  << "Within " << SUCCESS_MEDIUM_M << " m: " << successes_10m << '/' << targets.size() << '\n'
                  << "Within " << SUCCESS_FAR_M << " m: " << successes_25m << '/' << targets.size() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}

#include "tuning/pid_ga_tuner.hpp"
#include "ork/ork_loader.hpp"
#include "cli_args.hpp"
#include <iostream>

// ##### pid_ga_tuner, entry point #####
// Goal: load the rocket the same way main.cpp/gain_tuner.cpp already do
// (no parameters hand-copied here), then hand off to the actual
// three-way genetic algorithm in tuning/pid_ga_runner.cpp. Not part of
// the flight computer -- an offline tool, run by hand, whose result you
// review and paste into NavigationStep's default gains yourself.
int main() {
    const std::string project_root = findProjectRoot();
    const std::string ork_path = project_root + "/../artifacts/rocket.ork";

    SimulationConfig config;
    config.dt = 0.001;  // preview-speed -- this search runs many thousands of short flights

    std::cout << "Loading rocket design from " << ork_path << "...\n";
    OrkRocket rocket = loadOrkRocket(ork_path, config.dt);
    config.launch_height = rocket.launch_conditions.launch_height;
    config.init_tilt = 0.0;
    config.init_yaw = 0.0;

    tuning::runAllThreeGAs(rocket.params, config, rocket.mass_components, rocket.flight_data);
    return 0;
}

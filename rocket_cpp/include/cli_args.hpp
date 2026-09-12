#pragma once

#include <string>

// ##### CliArgs / parseArgs() #####
// Goal: let a caller (a human, or scripts/trajectory.py's GUI) override
// main.cpp's launch target/tilt and switch to a faster preview fidelity,
// without editing source. Anything not passed keeps main.cpp's own
// hardcoded default, so running with no flags at all behaves exactly
// like before these existed. Split out of main.cpp (not part of the
// sim/ork/gnc module split -- this is a main.cpp-only utility, same as
// main.cpp itself sitting outside those folders) purely to keep
// main.cpp under this project's 300-line convention.
struct CliArgs {
    double target_x = 0.0, target_y = 350.0, target_z = 1500.0;
    double init_tilt_deg = 0.0;
    double init_yaw_deg = 10.0;
    bool preview = false;  // faster/coarser: bigger dt, shorter duration cap -- see main.cpp
    bool no_plot = false;  // skip main.cpp's system("python3 scripts/plot_trajectory.py ...") call
};

CliArgs parseArgs(int argc, char** argv);

// ##### findProjectRoot() #####
// Goal: work out where THIS repo actually lives on disk, instead of
// hardcoding one machine's own absolute path -- reads /proc/self/exe
// (the running binary's own real, resolved location, regardless of how
// it was invoked or what the current directory happens to be) and walks
// up from there. The executable always lives at <repo_root>/rocket_cpp/
// build/rocket_cpp, so two levels up from its own directory is the repo
// root.
std::string findProjectRoot();

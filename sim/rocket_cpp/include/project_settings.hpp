#pragma once

// Paths are relative to sim/rocket_cpp, as returned by findProjectRoot().
// Shared by the simulator, sweep, and offline tuner.
namespace project_paths {
inline constexpr char ROCKET_DESIGN[] = "/../artifacts/rocket.ork";
inline constexpr char TRAJECTORY_CSV[] = "rocket_trajectory.csv";
inline constexpr char ANALYSIS_PNG[] = "rocket_analysis.png";
inline constexpr char TRAJECTORY_PNG[] = "rocket_trajectory.png";
inline constexpr char PLOT_SCRIPT[] = "scripts/plot_trajectory.py";
inline constexpr char PYTHON_EXECUTABLE[] = "python3";
inline constexpr char SWEEP_BASELINE_CSV[] = "/results/csv/interception_results_baseline.csv";
inline constexpr char SWEEP_TERMINAL_CSV[] = "/results/csv/interception_results_terminal.csv";
inline constexpr char EXECUTABLE_LINK[] = "/proc/self/exe";
inline constexpr char EXECUTABLE_TO_PROJECT[] = "/..";
inline constexpr char WORKING_DIRECTORY[] = ".";
}

namespace simulation_settings {
inline constexpr double PREVIEW_DT_S = 0.004;
inline constexpr double FULL_DT_S = 0.001;
inline constexpr double PREVIEW_DURATION_S = 60.0;
inline constexpr double FULL_DURATION_S = 300.0;
inline constexpr double DIAGNOSTIC_ALTITUDE_M = 100.0;
inline constexpr double DIAGNOSTIC_SPEED_MPS = 50.0;
inline constexpr double DIAGNOSTIC_ALPHA_RAD = 0.05;
inline constexpr int CSV_PRECISION = 6;
}

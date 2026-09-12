#include "cli_args.hpp"
#include <unistd.h>
#include <climits>

// ##### parseArgs() #####
// Goal: read optional flags off argv, falling back to CliArgs' own
// defaults for anything not given.
CliArgs parseArgs(int argc, char** argv) {
    CliArgs args;
    for (int i = 1; i < argc; ++i) {
        std::string flag = argv[i];
        if (flag == "--target" && i + 3 < argc) {
            args.target_x = std::stod(argv[++i]);
            args.target_y = std::stod(argv[++i]);
            args.target_z = std::stod(argv[++i]);
        } else if (flag == "--init-tilt" && i + 1 < argc) {
            args.init_tilt_deg = std::stod(argv[++i]);
        } else if (flag == "--init-yaw" && i + 1 < argc) {
            args.init_yaw_deg = std::stod(argv[++i]);
        } else if (flag == "--preview") {
            args.preview = true;
        } else if (flag == "--no-plot") {
            args.no_plot = true;
        }
    }
    return args;
}

// ##### findProjectRoot() #####
// Goal: see this function's own doc comment in cli_args.hpp.
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

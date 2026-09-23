#include "cli_args.hpp"
#include "simulation_runner.hpp"

#include <iostream>
#include <stdexcept>

// Keep process-level concerns here: parse the CLI, translate exceptions
// into a clean error code, and delegate the simulation itself to the
// reusable runner module.
int main(int argc, char** argv) {
    try {
        return runSimulation(parseArgs(argc, argv));
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}

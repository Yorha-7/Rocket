#pragma once

#include "cli_args.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

// Read a time-segmented TVC test CSV and expand it to one direction per
// simulation step.
std::vector<Eigen::Vector3d> loadTvcTestSequence(const std::string& csv_path,
                                                 double dt,
                                                 int n_steps);

// Run one complete guided simulation and write its trajectory/plots.
int runSimulation(const CliArgs& args);

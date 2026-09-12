#pragma once

#include "sim/rocket_types.hpp"
#include <string>

// ##### parseOrkFlightData() #####
// Goal: read one flight's motor THRUST CURVE out of an OpenRocket
// design's own embedded simulation results, resampled onto a uniform dt
// grid -- the one thing this codebase still trusts OpenRocket for, since
// there's no independent motor database to derive it from. An .ork can
// hold several embedded simulations, one per motor; if motor_configid is
// left empty, the design's own default="true" motor is used.
FlightData parseOrkFlightData(const std::string& xml, double dt,
                              const std::string& motor_configid = "");

// ##### parseOrkLaunchConditions() #####
// Goal: read the launch conditions (pad height, initial tilt) OpenRocket
// itself used for the SAME simulation parseOrkFlightData would pick, so
// main.cpp's own SimulationConfig matches the flight data it's paired with.
SimulationConfig parseOrkLaunchConditions(const std::string& xml,
                                          const std::string& motor_configid = "");

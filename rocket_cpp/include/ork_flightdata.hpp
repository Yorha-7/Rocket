#pragma once

#include "rocket_types.hpp"
#include <string>

// Reads one flight's time history out of an OpenRocket design XML: the
// motor's thrust curve, the vehicle's mass, drag coefficient, and its
// real (Barrowman-computed) CP/CG/inertia — all as they change over
// time, resampled onto a uniform dt grid, instead of guessed constants.
//
// An .ork file can hold several embedded simulations, one per motor. If
// motor_configid is left empty, the motor configuration marked
// default="true" in the design is used.
//
// motor_configid = "" is a default argument -- callers may omit it
// entirely (parseOrkFlightData(xml, dt)) and this value is used instead.
FlightData parseOrkFlightData(const std::string& xml, double dt,
                              const std::string& motor_configid = "");

// Reads the launch conditions (pad height, initial tilt from vertical)
// OpenRocket used for the same simulation parseOrkFlightData would pick,
// so main.cpp's SimulationConfig matches the flight data it's paired with.
SimulationConfig parseOrkLaunchConditions(const std::string& xml,
                                          const std::string& motor_configid = "");

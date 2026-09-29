#pragma once

#include "sim/rocket_types.hpp"
#include "ork/ork_mass_components.hpp"
#include <string>
#include <vector>

// ##### OrkRocket #####
// Goal: bundle everything main.cpp needs to run a simulation, all read
// straight out of one OpenRocket .ork design file.
struct OrkRocket {
    RocketParams params;                     // fixed geometry
    SimulationConfig launch_conditions;      // pad height / initial tilt this motor was launched at
    FlightData flight_data;                  // thrust/mass over time (motor performance data)
    std::vector<MassComponent> mass_components;  // structural parts, for VehicleMassModel
};

// ##### loadOrkRocket() #####
// Goal: the one call site -- load geometry, flight-data history, and
// structural mass components from an .ork file together. If
// motor_configid is empty, uses whichever motor configuration the
// design itself marks as default.
OrkRocket loadOrkRocket(const std::string& ork_path, double dt,
                        const std::string& motor_configid = "");

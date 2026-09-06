#pragma once

#include "rocket_types.hpp"
#include "ork_mass_components.hpp"
#include <string>
#include <vector>

// Everything main.cpp needs to run a simulation, read straight out of an
// OpenRocket .ork design file.
struct OrkRocket {
    RocketParams params;                     // fixed geometry
    SimulationConfig launch_conditions;      // pad height / initial tilt this motor was launched at
    FlightData flight_data;                  // thrust/mass over time (motor performance data)
    std::vector<MassComponent> mass_components;  // structural parts, for VehicleMassModel
};

// Loads geometry, flight-data history, and structural mass components from
// an .ork file in one call. If motor_configid is empty, uses whichever
// motor configuration the design marks as default.
OrkRocket loadOrkRocket(const std::string& ork_path, double dt,
                        const std::string& motor_configid = "");

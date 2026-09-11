#include "ork/ork_loader.hpp"
#include "ork/ork_archive.hpp"
#include "ork/ork_geometry.hpp"
#include "ork/ork_flightdata.hpp"
#include "ork/ork_mass_components.hpp"

OrkRocket loadOrkRocket(const std::string& ork_path, double dt,
                        const std::string& motor_configid) {
    std::string xml = readOrkXml(ork_path);

    OrkRocket rocket;
    rocket.params = parseOrkGeometry(xml);
    rocket.flight_data = parseOrkFlightData(xml, dt, motor_configid);
    rocket.launch_conditions = parseOrkLaunchConditions(xml, motor_configid);
    rocket.mass_components = parseOrkMassComponents(xml);
    return rocket;
}

#include "ork/ork_flightdata.hpp"
#include <tinyxml2.h>
#include <stdexcept>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <vector>

using tinyxml2::XMLElement;
using tinyxml2::XMLDocument;

namespace {

// Which motor configuration OpenRocket itself considers "the" one, when
// the caller doesn't ask for a specific motor.
std::string findDefaultMotorConfigId(const XMLElement* rocket) {
    for (const XMLElement* mc = rocket->FirstChildElement("motorconfiguration"); mc;
         mc = mc->NextSiblingElement("motorconfiguration")) {
        const char* is_default = mc->Attribute("default");
        if (is_default && std::string(is_default) == "true" && mc->Attribute("configid")) {
            return mc->Attribute("configid");
        }
    }
    const XMLElement* first = rocket->FirstChildElement("motorconfiguration");
    if (first && first->Attribute("configid")) return first->Attribute("configid");
    throw std::runtime_error("No motor configuration found in .ork file");
}

// The embedded simulation whose launch conditions were run with the given
// motor configuration. OpenRocket can save MULTIPLE <simulation> entries
// for the same config id -- e.g. an old "loaded but never re-run" one
// left over from before a rocket edit, sitting right next to the current
// one that's actually been simulated. Picking the first match blindly
// grabs whichever one happens to come first in the file, data or not; a
// simulation only being useful here once it actually HAS flight data
// (<flightdata><databranch>) is the real criterion, not just "first name
// match". Falls back to the first match with no data if none have any,
// so a genuinely never-simulated file still fails with the same honest
// "no flight data" error as before -- this only changes behavior when a
// good one actually exists among several candidates.
const XMLElement* findSimulationForConfig(const XMLElement* root, const std::string& configid) {
    const XMLElement* simulations = root->FirstChildElement("simulations");
    if (!simulations) return nullptr;

    const XMLElement* first_match = nullptr;
    for (const XMLElement* sim = simulations->FirstChildElement("simulation"); sim;
         sim = sim->NextSiblingElement("simulation")) {
        const XMLElement* conditions = sim->FirstChildElement("conditions");
        const XMLElement* id_elem = conditions ? conditions->FirstChildElement("configid") : nullptr;
        if (!id_elem || !id_elem->GetText() || configid != id_elem->GetText()) continue;

        if (!first_match) first_match = sim;

        const XMLElement* flightdata = sim->FirstChildElement("flightdata");
        const XMLElement* databranch = flightdata ? flightdata->FirstChildElement("databranch") : nullptr;
        if (databranch && databranch->Attribute("types")) {
            return sim;  // has real data -- this is the one we want
        }
    }
    return first_match;
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ',')) tokens.push_back(token);
    return tokens;
}

// Same "search the header text for this label" approach the old CSV
// loader used to find columns by name instead of a fixed index.
int columnIndex(const std::vector<std::string>& columns, const std::string& label) {
    for (size_t i = 0; i < columns.size(); ++i) {
        if (columns[i] == label) return static_cast<int>(i);
    }
    return -1;
}

// NaN shows up for columns that don't apply yet (e.g. CP location before
// the rocket has any airspeed) — treated as "unknown", not a real zero.
double toDoubleOrZero(const std::string& s) {
    try {
        double v = std::stod(s);
        return std::isnan(v) ? 0.0 : v;
    } catch (...) {
        return 0.0;
    }
}

// Nearest-neighbor resample onto a uniform dt grid, same approach the
// original CSV loader used.
std::vector<double> resample(const std::vector<double>& raw_time,
                             const std::vector<double>& raw_values,
                             const std::vector<double>& grid_time) {
    std::vector<double> out(grid_time.size());
    for (size_t i = 0; i < grid_time.size(); ++i) {
        double t = grid_time[i];
        auto it = std::lower_bound(raw_time.begin(), raw_time.end(), t);
        size_t idx;
        if (it == raw_time.begin()) {
            idx = 0;
        } else if (it == raw_time.end()) {
            idx = raw_time.size() - 1;
        } else {
            size_t idx1 = it - raw_time.begin();
            size_t idx0 = idx1 - 1;
            idx = (std::abs(raw_time[idx1] - t) < std::abs(raw_time[idx0] - t)) ? idx1 : idx0;
        }
        out[i] = raw_values[idx];
    }
    return out;
}

}  // namespace

FlightData parseOrkFlightData(const std::string& xml, double dt,
                              const std::string& motor_configid) {
    XMLDocument doc;
    if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to parse .ork XML");
    }

    const XMLElement* root = doc.RootElement();
    const XMLElement* rocket = root->FirstChildElement("rocket");
    if (!rocket) throw std::runtime_error(".ork file has no <rocket> element");

    std::string configid = motor_configid.empty() ? findDefaultMotorConfigId(rocket) : motor_configid;

    const XMLElement* simulation = findSimulationForConfig(root, configid);
    if (!simulation) throw std::runtime_error("No simulation found for motor config " + configid);

    const XMLElement* flightdata = simulation->FirstChildElement("flightdata");
    const XMLElement* databranch = flightdata ? flightdata->FirstChildElement("databranch") : nullptr;
    const char* types_attr = databranch ? databranch->Attribute("types") : nullptr;
    if (!types_attr) throw std::runtime_error("Simulation has no flight data to read");

    // Thrust and mass are motor performance data -- the one thing we still
    // read from OpenRocket's simulation, since there's no local motor
    // database to derive a thrust curve from independently. Cd/CP/CG/
    // inertia are no longer read here at all; AerodynamicsModel and
    // VehicleMassModel compute those from the vehicle's own geometry.
    std::vector<std::string> columns = splitCsv(types_attr);
    int time_col = columnIndex(columns, "Time");
    int thrust_col = columnIndex(columns, "Thrust");
    int mass_col = columnIndex(columns, "Mass");

    if (time_col < 0 || thrust_col < 0 || mass_col < 0) {
        throw std::runtime_error("Flight data is missing an expected column");
    }

    std::vector<double> raw_time, raw_thrust, raw_mass_g;

    for (const XMLElement* dp = databranch->FirstChildElement("datapoint"); dp;
         dp = dp->NextSiblingElement("datapoint")) {
        if (!dp->GetText()) continue;
        std::vector<std::string> fields = splitCsv(dp->GetText());
        if ((int)fields.size() <= std::max({time_col, thrust_col, mass_col})) {
            continue;
        }

        raw_time.push_back(toDoubleOrZero(fields[time_col]));
        raw_thrust.push_back(toDoubleOrZero(fields[thrust_col]));
        raw_mass_g.push_back(toDoubleOrZero(fields[mass_col]) * 1000.0);  // OpenRocket reports kg
    }

    if (raw_time.empty()) throw std::runtime_error("Simulation has no flight data points");

    FlightData data;
    double t_start = raw_time.front();
    double t_end = raw_time.back();
    int n_points = static_cast<int>((t_end - t_start) / dt) + 1;
    data.time.resize(n_points);
    for (int i = 0; i < n_points; ++i) data.time[i] = t_start + i * dt;

    data.thrust = resample(raw_time, raw_thrust, data.time);
    data.mass = resample(raw_time, raw_mass_g, data.time);

    double dry_mass = raw_mass_g.back();
    for (int i = (int)raw_thrust.size() - 1; i >= 0; --i) {
        if (raw_thrust[i] > 0.001) {
            dry_mass = raw_mass_g[i];
            break;
        }
    }
    data.dry_mass = dry_mass;

    return data;
}

SimulationConfig parseOrkLaunchConditions(const std::string& xml, const std::string& motor_configid) {
    XMLDocument doc;
    if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Failed to parse .ork XML");
    }

    const XMLElement* root = doc.RootElement();
    const XMLElement* rocket = root->FirstChildElement("rocket");
    if (!rocket) throw std::runtime_error(".ork file has no <rocket> element");

    std::string configid = motor_configid.empty() ? findDefaultMotorConfigId(rocket) : motor_configid;
    const XMLElement* simulation = findSimulationForConfig(root, configid);
    const XMLElement* conditions = simulation ? simulation->FirstChildElement("conditions") : nullptr;

    SimulationConfig config{};
    config.launch_height = 0.0;
    config.init_tilt = 0.0;

    if (conditions) {
        const XMLElement* alt = conditions->FirstChildElement("launchaltitude");
        if (alt && alt->GetText()) config.launch_height = std::stod(alt->GetText());

        // Stored in radians, like every other angle in an .ork file;
        // SimulationConfig.init_tilt is degrees.
        const XMLElement* rod_angle = conditions->FirstChildElement("launchrodangle");
        if (rod_angle && rod_angle->GetText()) {
            config.init_tilt = std::stod(rod_angle->GetText()) * 180.0 / M_PI;
        }
    }

    return config;
}

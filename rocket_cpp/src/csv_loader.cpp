#include "csv_loader.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

FlightData loadFlightData(const std::string& csv_path, double dt) {
    FlightData data;
    
    std::ifstream file(csv_path);
    if (!file.is_open()) {
        return data;
    }
    
    std::string line;
    bool header_parsed = false;
    int thrust_col = -1, mass_col = -1, cd_col = -1, time_col = -1;
    
    std::vector<double> raw_time, raw_thrust, raw_mass, raw_cd;
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            if (line.find("Time (s)") != std::string::npos) {
                // Parse header to find column indices
                std::stringstream ss(line);
                std::string token;
                int col = 0;
                while (std::getline(ss, token, ',')) {
                    if (token.find("Time") != std::string::npos) time_col = col;
                    else if (token.find("Thrust") != std::string::npos) thrust_col = col;
                    else if (token.find("Mass") != std::string::npos) mass_col = col;
                    else if (token.find("Drag coefficient") != std::string::npos) cd_col = col;
                    col++;
                }
                header_parsed = true;
            }
            continue;
        }
        
        if (!header_parsed) continue;
        
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() < 8) continue;
        
        double t = std::stod(tokens[time_col]);
        double thrust_val = std::stod(tokens[thrust_col]);
        double mass_val = std::stod(tokens[mass_col]);
        double cd_val = (cd_col >= 0 && cd_col < (int)tokens.size()) ? std::stod(tokens[cd_col]) : 0.867734;
        
        // Handle NaN values
        if (std::isnan(cd_val)) cd_val = 0.867734;
        
        raw_time.push_back(t);
        raw_thrust.push_back(thrust_val);
        raw_mass.push_back(mass_val);
        raw_cd.push_back(cd_val);
    }
    
    if (raw_time.empty()) return data;
    
    // Find dry mass (mass at last thrust point)
    double dry_mass = 0.0;
    for (int i = raw_thrust.size() - 1; i >= 0; --i) {
        if (raw_thrust[i] > 0.001) {
            dry_mass = raw_mass[i];
            break;
        }
    }
    if (dry_mass == 0.0 && !raw_mass.empty()) {
        dry_mass = raw_mass.back();
    }
    data.dry_mass = dry_mass;
    
    // Resample to uniform timestep using nearest neighbor
    double t_start = raw_time.front();
    double t_end = raw_time.back();
    int n_points = static_cast<int>((t_end - t_start) / dt) + 1;
    
    data.time.resize(n_points);
    data.thrust.resize(n_points);
    data.mass.resize(n_points);
    data.drag_coeff.resize(n_points);
    
    for (int i = 0; i < n_points; ++i) {
        double t = t_start + i * dt;
        data.time[i] = t;
        
        // Nearest neighbor search
        auto it = std::lower_bound(raw_time.begin(), raw_time.end(), t);
        size_t idx;
        if (it == raw_time.begin()) {
            idx = 0;
        } else if (it == raw_time.end()) {
            idx = raw_time.size() - 1;
        } else {
            size_t idx1 = it - raw_time.begin();
            size_t idx0 = idx1 - 1;
            double diff1 = std::abs(raw_time[idx1] - t);
            double diff0 = std::abs(raw_time[idx0] - t);
            idx = (diff1 < diff0) ? idx1 : idx0;
        }
        
        data.thrust[i] = raw_thrust[idx];
        data.mass[i] = raw_mass[idx];
        data.drag_coeff[i] = raw_cd[idx];
    }
    
    return data;
}

std::vector<double> generateThrustProfile(int n_points, double dt) {
    std::vector<double> thrust(n_points, 0.0);
    for (int i = 0; i < n_points; ++i) {
        double t = i * dt;
        if (t < 0.01) {
            thrust[i] = 0.0;
        } else if (t < 0.081) {
            double frac = (t - 0.01) / (0.081 - 0.01);
            thrust[i] = 0.305 + frac * (4.126 - 0.305);
        } else if (t < 1.86) {
            double frac = (t - 0.081) / (1.86 - 0.081);
            thrust[i] = 4.126 * (1.0 - frac);
        } else {
            thrust[i] = 0.0;
        }
    }
    return thrust;
}

std::vector<double> generateDragCoeffs(int n_points, double value) {
    return std::vector<double>(n_points, value);
}

std::vector<double> generateNormalCoeffs(int n_points, double value) {
    return std::vector<double>(n_points, value);
}

double getMassAtTime(const FlightData& data, double time) {
    if (data.time.empty() || data.mass.empty()) return 0.0;
    
    auto it = std::lower_bound(data.time.begin(), data.time.end(), time);
    if (it == data.time.begin()) return data.mass.front();
    if (it == data.time.end()) return data.mass.back();
    
    size_t idx = it - data.time.begin();
    return data.mass[idx];
}

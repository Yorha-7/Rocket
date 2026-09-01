#include "csv_loader.hpp"
#include <vector>
#include <algorithm>

FlightData loadFlightData(const std::string& csv_path, double dt) {
    // For now, return empty - will be implemented with actual CSV parsing
    // The main.cpp generates synthetic data based on known CSV values
    FlightData data;
    return data;
}

std::vector<double> generateThrustProfile(int n_points, double dt) {
    std::vector<double> thrust(n_points, 0.0);

    for (int i = 0; i < n_points; ++i) {
        double t = i * dt;
        if (t < 0.01) {
            thrust[i] = 0.0;
        } else if (t < 0.081) {
            // Linear ramp up from CSV data
            double frac = (t - 0.01) / (0.081 - 0.01);
            thrust[i] = 0.305 + frac * (4.126 - 0.305);
        } else if (t < 1.86) {
            // Sustained burn decreasing to 0
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
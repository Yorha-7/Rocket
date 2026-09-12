#include "sim/aerodynamics.hpp"
#include <cmath>
#include <algorithm>

// ##### Atmosphere + flow helpers, the big picture #####
// Goal: one shared air model (ISA -- troposphere below 11km, isothermal
// stratosphere above) and the flow quantities (Mach, Reynolds, skin
// friction) everything else in this file is built on, so there's only
// ever one air-density formula in play across the whole sim, not a
// second inline approximation hiding somewhere else.

// Goal: air temperature at a given altitude -- linear lapse below the
// tropopause, constant above it (isothermal layer).
double AerodynamicsModel::getTemperature(double altitude) const {
    if (altitude <= TROPOPAUSE_ALT) {
        return T0 + LAPSE_RATE * altitude;
    }
    return TROPOPAUSE_TEMP;
}

// Goal: air density at a given altitude -- power-law pressure formula in
// the troposphere (temperature drops linearly there), true exponential
// decay above it (temperature is constant there, so the barometric
// formula simplifies to a pure exponential). Ideal gas law converts
// pressure back to density in both cases.
double AerodynamicsModel::getDensity(double altitude) const {
    double exponent = -G0 / (LAPSE_RATE * R_GAS);
    if (altitude <= TROPOPAUSE_ALT) {
        double T = getTemperature(altitude);
        double P = P0 * std::pow(T / T0, exponent);
        return P / (R_GAS * T);
    }
    double T_tp = TROPOPAUSE_TEMP;
    double P_tp = P0 * std::pow(T_tp / T0, exponent);
    double P = P_tp * std::exp(-G0 * (altitude - TROPOPAUSE_ALT) / (R_GAS * T_tp));
    return P / (R_GAS * T_tp);
}

// Goal: Sutherland's law -- gas viscosity RISES with temperature (the
// opposite of most liquids), since gas viscosity comes from molecules
// exchanging momentum, not from intermolecular friction.
double AerodynamicsModel::getViscosity(double altitude) const {
    double T = getTemperature(altitude);
    return MU0 * (T0_SUTH + SUTHERLAND_CONST) / (T + SUTHERLAND_CONST) * std::pow(T / T0_SUTH, 1.5);
}

double AerodynamicsModel::getSpecificHeatRatio(double /*altitude*/) const {
    return 1.4;  // air, essentially constant across this flight envelope
}

double AerodynamicsModel::getSpeedOfSound(double altitude) const {
    return std::sqrt(getSpecificHeatRatio(altitude) * R_GAS * getTemperature(altitude));
}

double AerodynamicsModel::computeMach(double velocity, double altitude) const {
    return velocity / getSpeedOfSound(altitude);
}

double AerodynamicsModel::computeReynolds(double velocity, double altitude, double length) const {
    return getDensity(altitude) * velocity * length / getViscosity(altitude);
}

// Goal: find the Reynolds number above which surface roughness starts to
// dominate the boundary layer -- below it, treat the surface as
// effectively smooth instead.
double AerodynamicsModel::computeTransitionReynolds(double roughness, double length) const {
    if (roughness <= 0.0) return 1e30;  // effectively "never" -- perfectly smooth surface
    return 51.0 * std::pow(roughness / length, -1.039);
}

// Goal: flat-plate skin friction coefficient -- laminar (Blasius) below
// Re=1e4, turbulent (5th-power law) above it, capped by the fully-rough-
// flow limit once surface roughness takes over. Standard Barrowman/
// Mandell treatment, what this file's whole friction-drag model rests on.
double AerodynamicsModel::computeSkinFrictionCf(double Re, double roughness, double length) const {
    if (Re <= 0.0) return 0.0;

    double Cf_smooth = (Re < 1.0e4) ? 1.328 / std::sqrt(Re) : 0.074 / std::pow(Re, 0.2);

    double Re_crit = computeTransitionReynolds(roughness, length);
    if (Re <= Re_crit) return Cf_smooth;

    double Cf_rough = std::pow(1.89 + 1.62 * std::log10(length / roughness), -2.5);
    return std::max(Cf_smooth, Cf_rough);
}

// Goal: total air-touching surface area -- nose cone (approximated as a
// simple cone regardless of its exact shape, fine for a friction-drag
// estimate), body tube, and both sides of every fin.
double AerodynamicsModel::computeWettedArea() const {
    double r = params_.body_diameter / 2.0;
    double slant = std::sqrt(params_.nose_length * params_.nose_length + r * r);
    double nose_area = M_PI * r * slant;

    double body_area = 2.0 * M_PI * r * params_.body_length;

    double fin_area_per_side = 0.5 * (params_.fin_root_chord + params_.fin_tip_chord) * params_.fin_span;
    double fin_area = 2.0 * fin_area_per_side * params_.fin_count;

    return nose_area + body_area + fin_area;
}

// Goal: whole-vehicle friction drag -- skin friction coefficient times
// wetted area, with a slender-body correction since a real body isn't a
// perfectly flat plate (friction drag runs a little higher than Cf alone
// predicts).
double AerodynamicsModel::computeFrictionDrag(const FlightConditions& fc) const {
    double length = params_.nose_length + params_.body_length;
    double Re = computeReynolds(fc.velocity, fc.altitude, length);
    double Cf = computeSkinFrictionCf(Re, params_.surface_roughness, length);

    double fineness = length / params_.body_diameter;
    double form_factor = 1.0 + 0.5 / fineness;

    return Cf * form_factor * computeWettedArea() / params_.reference_area;
}

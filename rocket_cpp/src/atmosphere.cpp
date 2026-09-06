#include "aerodynamics.hpp"
#include <cmath>
#include <algorithm>

// ============================================================
// Standard atmosphere (ISA, troposphere + isothermal stratosphere) and
// the flow quantities (Mach, Reynolds, skin friction) everything else in
// this file is built on. Shared by the whole sim so there's only ever one
// air-density model in play, not a second inline approximation elsewhere.
// ============================================================

double AerodynamicsModel::getTemperature(double altitude) const {
    if (altitude <= TROPOPAUSE_ALT) {
        return T0 + LAPSE_RATE * altitude;
    }
    return TROPOPAUSE_TEMP;
}

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

// Sutherland's law -- viscosity rises with temperature, the opposite of
// most liquids, because gas viscosity comes from molecular momentum
// exchange rather than intermolecular friction.
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

// Below this Reynolds number, surface roughness doesn't yet dominate the
// boundary layer -- below it, use the smooth-flow formula instead.
double AerodynamicsModel::computeTransitionReynolds(double roughness, double length) const {
    if (roughness <= 0.0) return 1e30;  // effectively "never" -- perfectly smooth surface
    return 51.0 * std::pow(roughness / length, -1.039);
}

// Flat-plate skin friction: laminar (Blasius) below Re=1e4, turbulent
// (5th-power law) above it, capped by the fully-rough-flow limit once
// surface roughness starts to dominate -- the standard Barrowman/Mandell
// treatment this codebase's whole friction-drag model is based on.
double AerodynamicsModel::computeSkinFrictionCf(double Re, double roughness, double length) const {
    if (Re <= 0.0) return 0.0;

    double Cf_smooth = (Re < 1.0e4) ? 1.328 / std::sqrt(Re) : 0.074 / std::pow(Re, 0.2);

    double Re_crit = computeTransitionReynolds(roughness, length);
    if (Re <= Re_crit) return Cf_smooth;

    double Cf_rough = std::pow(1.89 + 1.62 * std::log10(length / roughness), -2.5);
    return std::max(Cf_smooth, Cf_rough);
}

// Total wetted (air-touching) surface area: nose cone lateral surface
// (approximated as a cone regardless of exact shape -- fine for a
// friction-drag estimate), body tube lateral surface, and both sides of
// each fin's planform.
double AerodynamicsModel::computeWettedArea() const {
    double r = params_.body_diameter / 2.0;
    double slant = std::sqrt(params_.nose_length * params_.nose_length + r * r);
    double nose_area = M_PI * r * slant;

    double body_area = 2.0 * M_PI * r * params_.body_length;

    double fin_area_per_side = 0.5 * (params_.fin_root_chord + params_.fin_tip_chord) * params_.fin_span;
    double fin_area = 2.0 * fin_area_per_side * params_.fin_count;

    return nose_area + body_area + fin_area;
}

double AerodynamicsModel::computeFrictionDrag(const FlightConditions& fc) const {
    double length = params_.nose_length + params_.body_length;
    double Re = computeReynolds(fc.velocity, fc.altitude, length);
    double Cf = computeSkinFrictionCf(Re, params_.surface_roughness, length);

    // Slender-body form factor: a body isn't a flat plate, so its
    // friction drag is a little higher than Cf alone predicts.
    double fineness = length / params_.body_diameter;
    double form_factor = 1.0 + 0.5 / fineness;

    return Cf * form_factor * computeWettedArea() / params_.reference_area;
}

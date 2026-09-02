#pragma once

#include "rocket_types.hpp"

class AerodynamicsModel {
public:
    AerodynamicsModel(const RocketParams& params);
    
    // Main entry point
    AerodynamicCoefficients computeCoefficients(const FlightConditions& fc) const;
    
private:
    const RocketParams& params_;
    
    // ---- Body contributions ----
    double computeBodyCd(const FlightConditions& fc) const;
    double computeBodyCnAlpha() const;
    double computeBodyCmAlpha() const;
    
    // ---- Fin contributions ----
    double computeFinCd(const FlightConditions& fc) const;
    double computeFinCnAlpha() const;
    double computeFinCmAlpha() const;
    
    // ---- Base/Boat-tail drag ----
    double computeBaseDrag(const FlightConditions& fc) const;
    double computeBoatTailDrag(const FlightConditions& fc) const;
    
    // ---- Wave drag (supersonic) ----
    double computeWaveDrag(const FlightConditions& fc) const;
    
    // ---- Friction drag ----
    double computeFrictionDrag(const FlightConditions& fc) const;
    
    // ---- Interference effects ----
    double computeBodyFinInterference(const FlightConditions& fc) const;
    
    // ---- Helpers ----
    double computeReynolds(double velocity, double altitude, double length) const;
    double computeMach(double velocity, double altitude) const;
    double getSpecificHeatRatio(double altitude) const;
    double getViscosity(double altitude) const;
    double getTemperature(double altitude) const;
    double getDensity(double altitude) const;
    double getSpeedOfSound(double altitude) const;
    double computeSkinFrictionCf(double Re, double Mach, double roughness, double length) const;
    double computeTransitionReynolds(double Mach, double roughness, double length) const;
    double computeTransitionLocation(const FlightConditions& fc) const;
    double computeBodyCdSubsonic(const FlightConditions& fc) const;
    double computeBodyCdTransonic(const FlightConditions& fc) const;
    double computeBodyCdSupersonic(const FlightConditions& fc) const;
    double computeFinCd(const FlightConditions& fc) const;
    double computeFinCnAlpha() const;
    double computeFinCmAlpha() const;
    double computeBaseDrag(const FlightConditions& fc) const;
    double computeBoatTailDrag(const FlightConditions& fc) const;
    double computeWaveDrag(const FlightConditions& fc) const;
    double computeFrictionDrag(const FlightConditions& fc) const;
    double computeBodyFinInterference(const FlightConditions& fc) const;
    double getDensity(double altitude) const;
    double getSpeedOfSound(double altitude) const;
    double getTemperature(double altitude) const;
    double getViscosity(double altitude) const;
    double computeSkinFrictionCf(double Re, double Mach, double roughness, double length) const;
    double computeTransitionReynolds(double Mach, double roughness, double length) const;
    double computeTransitionLocation(const FlightConditions& fc) const;
    double computeBodyCdSubsonic(const FlightConditions& fc) const;
    double computeBodyCdTransonic(const FlightConditions& fc) const;
    double computeBodyCdSupersonic(const FlightConditions& fc) const;
    double computeFinCd(const FlightConditions& fc) const;
    double computeFinCnAlpha() const;
    double computeFinCmAlpha() const;
    double computeBaseDrag(const FlightConditions& fc) const;
    double computeBoatTailDrag(const FlightConditions& fc) const;
    double computeWaveDrag(const FlightConditions& fc) const;
    double computeFrictionDrag(const FlightConditions& fc) const;
    double computeBodyFinInterference(const FlightConditions& fc) const;
    double getDensity(double altitude) const;
    double getSpeedOfSound(double altitude) const;
    double getTemperature(double altitude) const;
    double getViscosity(double altitude) const;
    double computeSkinFrictionCf(double Re, double Mach, double roughness, double length) const;
    double computeTransitionReynolds(double Mach, double roughness, double length) const;
    double computeTransitionLocation(const FlightConditions& fc) const;
    double computeBodyCdSubsonic(const FlightConditions& fc) const;
    double computeBodyCdTransonic(const FlightConditions& fc) const;
    double computeBodyCdSupersonic(const FlightConditions& fc) const;
    double computeFinCd(const FlightConditions& fc) const;
    double computeFinCnAlpha() const;
    double computeFinCmAlpha() const;
    double computeBaseDrag(const FlightConditions& fc) const;
    double computeBoatTailDrag(const FlightConditions& fc) const;
    double computeWaveDrag(const FlightConditions& fc) const;
    double computeFrictionDrag(const FlightConditions& fc) const;
    double computeBodyFinInterference(const FlightConditions& fc) const;
    
    // Standard atmosphere constants
    static constexpr double R_GAS = 287.058;  // J/(kg*K)
    static constexpr double G0 = 9.80665;
    static constexpr double R_EARTH = 6371000.0;
    static constexpr double T0 = 288.15;
    static constexpr double P0 = 101325.0;
    static constexpr double RHO0 = 1.225;
    static constexpr double LAPSE_RATE = -0.0065;
    static constexpr double TROPOPAUSE_ALT = 11000.0;
    static constexpr double TROPOPAUSE_TEMP = 216.65;
    static constexpr double SUTHERLAND_CONST = 110.4;
    static constexpr double MU0 = 1.7894e-5;
    static constexpr double T0_SUTH = 273.15;
};
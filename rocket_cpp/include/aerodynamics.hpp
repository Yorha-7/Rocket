#pragma once

#include "rocket_types.hpp"

// Computes drag/normal-force coefficients and the center of pressure from
// the vehicle's own geometry (Barrowman method) -- nothing here is read
// from OpenRocket's own solved simulation. Implementation is split across
// aerodynamics.cpp (coefficients + Barrowman CP) and atmosphere.cpp
// (atmosphere model + Reynolds/skin-friction helpers), both methods of
// this one class.
class AerodynamicsModel {
public:
    explicit AerodynamicsModel(const RocketParams& params);

    // Main entry point: every drag/normal-force coefficient at one flight condition.
    AerodynamicCoefficients computeCoefficients(const FlightConditions& fc) const;

    // Barrowman center of pressure, cm from the nose tip. Geometry-only in
    // the subsonic regime -- doesn't need a FlightConditions.
    double computeCenterOfPressure() const;

    // Shared atmosphere model, public so the rest of the sim (translation,
    // pitch dynamics) uses the same air density/speed of sound instead of
    // a second, inconsistent formula.
    double getDensity(double altitude) const;
    double getSpeedOfSound(double altitude) const;

private:
    const RocketParams& params_;

    // ---- Body ----
    double computeBodyCd(const FlightConditions& fc) const;
    double computeBodyCdSubsonic(const FlightConditions& fc) const;
    double computeBodyCdTransonic(const FlightConditions& fc) const;
    double computeBodyCdSupersonic(const FlightConditions& fc) const;
    double computeBodyCnAlpha(double mach) const;
    double computeBodyCmAlpha(double cn_alpha_body) const;
    double computeNoseCp() const;

    // ---- Fins ----
    double computeFinCd(const FlightConditions& fc) const;
    double computeFinCnAlpha(double mach) const;
    double computeFinCmAlpha(double cn_alpha_fin) const;
    double computeFinCp() const;
    double computeBodyFinInterference() const;

    // ---- Drag components beyond body/fin form drag ----
    double computeBaseDrag(const FlightConditions& fc) const;
    double computeBoatTailDrag(const FlightConditions& fc) const;
    double computeWaveDrag(const FlightConditions& fc) const;
    double computeFrictionDrag(const FlightConditions& fc) const;
    double computeWettedArea() const;

    // ---- Atmosphere / flow helpers ----
    double getTemperature(double altitude) const;
    double getViscosity(double altitude) const;
    double getSpecificHeatRatio(double altitude) const;
    double computeMach(double velocity, double altitude) const;
    double computeReynolds(double velocity, double altitude, double length) const;
    double computeSkinFrictionCf(double Re, double roughness, double length) const;
    double computeTransitionReynolds(double roughness, double length) const;

    // Standard atmosphere constants
    static constexpr double R_GAS = 287.058;  // J/(kg*K)
    static constexpr double G0 = 9.80665;
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

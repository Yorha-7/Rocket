#pragma once

#include "sim/rocket_types.hpp"

// ##### AerodynamicsModel #####
// Goal: work out every drag/turning-force coefficient, and the center of
// pressure, straight from the vehicle's own shape (Barrowman method) --
// nothing here is read from OpenRocket's own solved simulation. Split
// across aerodynamics.cpp (coefficients + CP) and atmosphere.cpp (air
// model + Reynolds/skin-friction helpers), both part of this one class.
class AerodynamicsModel {
public:
    // explicit: stops the compiler from silently turning a RocketParams
    // into an AerodynamicsModel where that wasn't the intent.
    explicit AerodynamicsModel(const RocketParams& params);

    // Goal: the one call site that pulls every coefficient together for
    // a given flight condition.
    AerodynamicCoefficients computeCoefficients(const FlightConditions& fc) const;

    // Goal: the Barrowman center of pressure, cm from the nose tip.
    // Geometry-only in the subsonic regime -- doesn't need a
    // FlightConditions, so it's computed once and cached by the caller.
    double computeCenterOfPressure() const;

    // Goal: expose the shared atmosphere model so the rest of the sim
    // (translation, pitch/yaw dynamics) reads the SAME air density and
    // speed of sound this class uses internally, instead of each keeping
    // a second, possibly-inconsistent formula.
    double getDensity(double altitude) const;
    double getSpeedOfSound(double altitude) const;

private:
    const RocketParams& params_;  // reference, not a copy -- caller must outlive this object

    // ##### Body #####
    double computeBodyCd(const FlightConditions& fc) const;
    double computeBodyCdSubsonic(const FlightConditions& fc) const;
    double computeBodyCdTransonic(const FlightConditions& fc) const;
    double computeBodyCdSupersonic(const FlightConditions& fc) const;
    double computeBodyCnAlpha(double mach) const;
    double computeBodyCmAlpha(double cn_alpha_body) const;
    double computeNoseCp() const;

    // ##### Fins #####
    double computeFinCd(const FlightConditions& fc) const;
    double computeFinCnAlpha(double mach) const;
    double computeFinCmAlpha(double cn_alpha_fin) const;
    double computeFinCp() const;
    double computeBodyFinInterference() const;

    // ##### Drag beyond body/fin form drag #####
    double computeBaseDrag(const FlightConditions& fc) const;
    double computeBoatTailDrag(const FlightConditions& fc) const;
    double computeWaveDrag(const FlightConditions& fc) const;
    double computeFrictionDrag(const FlightConditions& fc) const;
    double computeWettedArea() const;

    // ##### Atmosphere / flow helpers (atmosphere.cpp) #####
    double getTemperature(double altitude) const;
    double getViscosity(double altitude) const;
    double getSpecificHeatRatio(double altitude) const;
    double computeMach(double velocity, double altitude) const;
    double computeReynolds(double velocity, double altitude, double length) const;
    double computeSkinFrictionCf(double Re, double roughness, double length) const;
    double computeTransitionReynolds(double roughness, double length) const;

    // Goal: one shared, fixed set of standard-atmosphere constants,
    // baked in at compile time (static constexpr) instead of re-typed
    // per method.
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

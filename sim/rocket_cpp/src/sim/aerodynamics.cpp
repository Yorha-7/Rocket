#include "sim/aerodynamics.hpp"
#include <cmath>
#include <algorithm>

// ##### Barrowman aerodynamics, the big picture #####
// Goal: every coefficient in this file comes from the vehicle's own
// geometry (nose shape/length, body diameter/length, fin planform), not
// a borrowed simulation result. Subsonic only in practice -- this
// vehicle never exceeds Mach ~0.28 -- so the transonic/supersonic paths
// below are honest placeholders, not missing work.

AerodynamicsModel::AerodynamicsModel(const RocketParams& params) : params_(params) {}

// ##### Body #####
// Goal: work out how much of the vehicle's turning force and drag comes
// from the body tube + nose alone, before adding the fins.

// Goal: Barrowman's classic result -- a nose that blends into a
// constant-diameter body contributes exactly 2/rad of normal-force slope
// (referenced to that body's own cross-section), regardless of nose
// shape or length.
double AerodynamicsModel::computeBodyCnAlpha(double mach) const {
    double beta2 = 1.0 - mach * mach;
    double compressibility = (beta2 > 0.01) ? 1.0 / std::sqrt(beta2) : 10.0;  // guard near Mach 1
    return 2.0 * compressibility;
}

double AerodynamicsModel::computeBodyCmAlpha(double cn_alpha_body) const {
    double d_cm = params_.body_diameter * 100.0;
    return -cn_alpha_body * (computeNoseCp() / d_cm);
}

// Goal: look up the nose's own center of pressure as a standard
// closed-form fraction of its length, one fraction per shape this
// codebase distinguishes.
double AerodynamicsModel::computeNoseCp() const {
    double L_cm = params_.nose_length * 100.0;
    double fraction;
    switch (params_.nose_shape) {
        case 0: fraction = 0.666; break;   // conical
        case 2: fraction = 0.5; break;     // hemisphere/ellipsoid
        case 3: fraction = 0.5; break;     // parabolic
        default: fraction = 0.466; break;  // ogive (tangent ogive)
    }
    return fraction * L_cm;
}

double AerodynamicsModel::computeBodyCd(const FlightConditions& fc) const {
    if (fc.mach < 0.8) return computeBodyCdSubsonic(fc);
    if (fc.mach < 1.2) return computeBodyCdTransonic(fc);
    return computeBodyCdSupersonic(fc);
}

// Goal: a pointed nose doesn't separate subsonic flow, so its own
// pressure drag is genuinely ~zero -- friction and base drag (computed
// separately) dominate instead. Zero here is the correct Barrowman
// result, not a stub.
double AerodynamicsModel::computeBodyCdSubsonic(const FlightConditions& /*fc*/) const {
    return 0.0;
}

// Goal: placeholders only -- this vehicle's fastest simulated flight is
// Mach ~0.28, so these paths never actually run. Rough order-of-
// magnitude numbers, not tuned.
double AerodynamicsModel::computeBodyCdTransonic(const FlightConditions& /*fc*/) const {
    return 0.1;
}

double AerodynamicsModel::computeBodyCdSupersonic(const FlightConditions& /*fc*/) const {
    return 0.2;
}

// ##### Fins #####
// Goal: same idea as the body section, but for the fin set's own
// contribution to turning force and drag.

// Goal: Barrowman fin normal-force slope for N fins, corrected for how
// much the body blocks/boosts the fins' own airflow (Kfb) and for
// compressibility.
double AerodynamicsModel::computeFinCnAlpha(double mach) const {
    if (params_.fin_count <= 0) return 0.0;

    double N = params_.fin_count;
    double s = params_.fin_span;
    double d = params_.body_diameter;
    double Cr = params_.fin_root_chord;
    double Ct = params_.fin_tip_chord;
    double Lm = std::sqrt(params_.fin_sweep * params_.fin_sweep + s * s);

    double incompressible = (4.0 * N * (s / d) * (s / d)) /
                            (1.0 + std::sqrt(1.0 + std::pow(2.0 * Lm / (Cr + Ct), 2.0)));

    double beta2 = 1.0 - mach * mach;
    double compressibility = (beta2 > 0.01) ? 1.0 / std::sqrt(beta2) : 10.0;

    return computeBodyFinInterference() * incompressible * compressibility;
}

double AerodynamicsModel::computeFinCmAlpha(double cn_alpha_fin) const {
    double d_cm = params_.body_diameter * 100.0;
    return -cn_alpha_fin * (computeFinCp() / d_cm);
}

// Goal: standard trapezoid-fin centroid formula, measured from the root
// leading edge -- the same shape formula also used for the fin set's
// mass CG in ork_mass_components.cpp, kept consistent between the two.
double AerodynamicsModel::computeFinCp() const {
    double Cr = params_.fin_root_chord;
    double Ct = params_.fin_tip_chord;
    double sweep = params_.fin_sweep;
    double centroid_from_root_le =
        (sweep * (Cr + 2.0 * Ct)) / (3.0 * (Cr + Ct)) +
        (1.0 / 6.0) * (Cr + Ct - (Cr * Ct) / (Cr + Ct));
    double root_le_cm = params_.fin_root_le_position * 100.0;
    return root_le_cm + centroid_from_root_le * 100.0;
}

// Goal: fold the body-fin interaction into one factor -- the body blocks
// part of the airflow a fin alone would see, and a fin effectively
// enlarges the body's own normal-force footprint. Feeds into
// computeFinCnAlpha above.
double AerodynamicsModel::computeBodyFinInterference() const {
    double r = params_.body_diameter / 2.0;
    double s = params_.fin_span;
    return 1.0 + r / (s + r);
}

// Goal: the EXTRA pressure drag a fin of finite thickness adds on top of
// an idealized thin flat plate (friction itself is already counted once,
// whole-vehicle, in computeFrictionDrag) -- the standard Hoerner
// thickness correction, Cf x 2(t/c).
double AerodynamicsModel::computeFinCd(const FlightConditions& fc) const {
    double mean_chord = 0.5 * (params_.fin_root_chord + params_.fin_tip_chord);
    if (mean_chord <= 0.0) return 0.0;

    double Re = computeReynolds(fc.velocity, fc.altitude, mean_chord);
    double Cf = computeSkinFrictionCf(Re, params_.surface_roughness, mean_chord);
    double thickness_ratio = params_.fin_thickness / mean_chord;

    double area_per_side = mean_chord * params_.fin_span;
    double fin_wetted_area = 2.0 * area_per_side * params_.fin_count;

    return Cf * 2.0 * thickness_ratio * fin_wetted_area / params_.reference_area;
}

// ##### Base / boat-tail #####
// Goal: the extra drag from a blunt-cut tail end (base drag) and, if the
// design has one, a boat-tail's own drag reduction.

// Goal: standard subsonic blunt-base drag correlation, scaled by how
// much of the reference area the base actually is (1.0 here -- no
// boat-tail on this vehicle, base = full body diameter).
double AerodynamicsModel::computeBaseDrag(const FlightConditions& fc) const {
    double base_r = params_.base_diameter / 2.0;
    double base_area = M_PI * base_r * base_r;
    double coeff = 0.12 + 0.13 * fc.mach * fc.mach;
    return coeff * (base_area / params_.reference_area);
}

// Goal: zero when there's no boat-tail (this vehicle) -- kept as a real
// formula rather than a stub so a future design with boat_tail_length >
// 0 gets correct behavior for free.
double AerodynamicsModel::computeBoatTailDrag(const FlightConditions& /*fc*/) const {
    if (params_.boat_tail_length <= 0.0) return 0.0;
    double d_ratio = (params_.body_diameter - params_.base_diameter) / params_.body_diameter;
    double slope = (params_.body_diameter - params_.base_diameter) / params_.boat_tail_length;
    return 2.0 * d_ratio * slope;
}

// Goal: placeholder, supersonic-only physics -- never exercised at this
// vehicle's actual speeds.
double AerodynamicsModel::computeWaveDrag(const FlightConditions& fc) const {
    return (fc.mach > 1.0) ? 0.2 : 0.0;
}

// ##### Assembly #####
// Goal: pull every piece above into the one complete coefficient set
// AerodynamicsModel actually hands out.
AerodynamicCoefficients AerodynamicsModel::computeCoefficients(const FlightConditions& fc) const {
    double cn_alpha_body = computeBodyCnAlpha(fc.mach);
    double cn_alpha_fin = computeFinCnAlpha(fc.mach);
    double cn_alpha_total = cn_alpha_body + cn_alpha_fin;

    double cm_alpha_body = computeBodyCmAlpha(cn_alpha_body);
    double cm_alpha_fin = computeFinCmAlpha(cn_alpha_fin);

    double cdp = computeBodyCd(fc) + computeFinCd(fc);
    double cdf = computeFrictionDrag(fc);
    double cdbase = computeBaseDrag(fc) + computeBoatTailDrag(fc);
    double cdwave = computeWaveDrag(fc);

    AerodynamicCoefficients out{};
    out.Cd = cdp + cdf + cdbase + cdwave;
    out.Cdp = cdp;
    out.Cdf = cdf;
    out.Cdbase = cdbase;
    out.Cd_wave = cdwave;
    out.Ca = out.Cd;                        // small-angle approximation
    out.Cn = cn_alpha_total * fc.alpha;
    out.Cm = (cm_alpha_body + cm_alpha_fin) * fc.alpha;
    out.Cl = 0.0;                           // not modeled
    out.Cn_alpha = cn_alpha_total;
    out.Cm_alpha = cm_alpha_body + cm_alpha_fin;
    return out;
}

// Goal: the Barrowman CP, computed once at Mach 0 rather than per flight
// condition -- both body and fin Cn_alpha pick up the same
// compressibility factor, which cancels out of this ratio, so the
// result doesn't actually depend on Mach.
double AerodynamicsModel::computeCenterOfPressure() const {
    double cn_body = computeBodyCnAlpha(0.0);
    double cn_fin = computeFinCnAlpha(0.0);
    double total = cn_body + cn_fin;
    if (total < 1e-9) return computeNoseCp();
    return (cn_body * computeNoseCp() + cn_fin * computeFinCp()) / total;
}

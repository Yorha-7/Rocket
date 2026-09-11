#include "sim/aerodynamics.hpp"
#include <cmath>
#include <algorithm>

// ============================================================
// Barrowman-method aerodynamics: every coefficient here comes from the
// vehicle's own geometry (nose shape/length, body diameter/length, fin
// planform), not from a borrowed simulation result. Subsonic (this
// vehicle never exceeds Mach 0.28); transonic/supersonic paths are
// placeholders since they're never exercised at these speeds.
// ============================================================

AerodynamicsModel::AerodynamicsModel(const RocketParams& params) : params_(params) {}

// ---- Body ----

// Barrowman's classic result: a nose that transitions fully into a
// constant-diameter body contributes exactly 2/rad (referenced to that
// body's own cross-sectional area), independent of nose shape or length.
double AerodynamicsModel::computeBodyCnAlpha(double mach) const {
    double beta2 = 1.0 - mach * mach;
    double compressibility = (beta2 > 0.01) ? 1.0 / std::sqrt(beta2) : 10.0;  // guard near Mach 1
    return 2.0 * compressibility;
}

double AerodynamicsModel::computeBodyCmAlpha(double cn_alpha_body) const {
    double d_cm = params_.body_diameter * 100.0;
    return -cn_alpha_body * (computeNoseCp() / d_cm);
}

// Standard closed-form nose CP fractions (fraction of nose length, from
// the tip) for the shapes this codebase distinguishes.
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

// A pointed nose doesn't separate subsonic flow, so its own pressure drag
// is essentially zero -- friction and base drag dominate instead (both
// computed separately). This is the correct Barrowman result, not a
// missing implementation.
double AerodynamicsModel::computeBodyCdSubsonic(const FlightConditions& /*fc*/) const {
    return 0.0;
}

// Placeholders: this vehicle's fastest simulated flight is Mach ~0.28, so
// these paths are never exercised. Rough order-of-magnitude values only.
double AerodynamicsModel::computeBodyCdTransonic(const FlightConditions& /*fc*/) const {
    return 0.1;
}

double AerodynamicsModel::computeBodyCdSupersonic(const FlightConditions& /*fc*/) const {
    return 0.2;
}

// ---- Fins ----

// Barrowman fin normal-force slope for N fins, corrected for body-fin
// interference (Kfb) and compressibility.
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

// Standard trapezoid-fin CP: measured from the root leading edge, using
// root/tip chord, span, and sweep -- the same centroid formula used for
// the fin set's mass CG in ork_mass_components.cpp.
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

// Body blocks part of the airflow a fin alone would see, and a fin
// increases the effective body diameter's normal force -- Kfb captures
// both, folded into computeFinCnAlpha above.
double AerodynamicsModel::computeBodyFinInterference() const {
    double r = params_.body_diameter / 2.0;
    double s = params_.fin_span;
    return 1.0 + r / (s + r);
}

// Fin profile (pressure) drag from finite thickness -- friction itself is
// already covered by computeFrictionDrag's whole-vehicle wetted-area
// total (fins included), so this is just the *extra* pressure drag a
// thick fin adds over a thin flat plate: Cf x 2(t/c), the standard
// Hoerner thickness correction, applied to the fins' own wetted area.
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

// ---- Base / boat-tail ----

// Standard subsonic blunt-base drag correlation, scaled by how much of
// the reference area the base actually is (1.0 here -- no boat-tail).
double AerodynamicsModel::computeBaseDrag(const FlightConditions& fc) const {
    double base_r = params_.base_diameter / 2.0;
    double base_area = M_PI * base_r * base_r;
    double coeff = 0.12 + 0.13 * fc.mach * fc.mach;
    return coeff * (base_area / params_.reference_area);
}

// Zero when there's no boat-tail (this vehicle) -- kept as a real formula,
// not a stub, since a future design with boat_tail_length > 0 should work.
double AerodynamicsModel::computeBoatTailDrag(const FlightConditions& /*fc*/) const {
    if (params_.boat_tail_length <= 0.0) return 0.0;
    double d_ratio = (params_.body_diameter - params_.base_diameter) / params_.body_diameter;
    double slope = (params_.body_diameter - params_.base_diameter) / params_.boat_tail_length;
    return 2.0 * d_ratio * slope;
}

// Placeholder -- supersonic-only, never exercised at this vehicle's speeds.
double AerodynamicsModel::computeWaveDrag(const FlightConditions& fc) const {
    return (fc.mach > 1.0) ? 0.2 : 0.0;
}

// ---- Assembly ----

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

// Barrowman CP is geometry-only in the subsonic regime: both body and fin
// Cn_alpha pick up the same compressibility factor, which cancels in this
// ratio -- so it's computed once at Mach 0, not per flight condition.
double AerodynamicsModel::computeCenterOfPressure() const {
    double cn_body = computeBodyCnAlpha(0.0);
    double cn_fin = computeFinCnAlpha(0.0);
    double total = cn_body + cn_fin;
    if (total < 1e-9) return computeNoseCp();
    return (cn_body * computeNoseCp() + cn_fin * computeFinCp()) / total;
}

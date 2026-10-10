#include "tuning/pid_ga_tuner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace tuning {


ParamSpec specFor(Param p) {
    switch (p) {
        case Param::KP: return {KP_MIN, KP_MAX, KP_BASELINE};
        case Param::KI: return {KI_MIN, KI_MAX, KI_BASELINE};
        case Param::KD: return {KD_MIN, KD_MAX, KD_BASELINE};
    }
    return {0.0, 1.0, 0.0};  // unreachable
}

std::string paramName(Param p) {
    switch (p) {
        case Param::KP: return "Kp";
        case Param::KI: return "Ki";
        case Param::KD: return "Kd";
    }
    return "?";  // unreachable
}

double decode(Chromosome c, const ParamSpec& spec) {
    return spec.min + (static_cast<double>(c) / static_cast<double>(GENE_MAX)) * (spec.max - spec.min);
}

Chromosome encode(double value, const ParamSpec& spec) {
    double clamped = std::max(spec.min, std::min(spec.max, value));
    double frac = (spec.max > spec.min) ? (clamped - spec.min) / (spec.max - spec.min) : 0.0;
    return static_cast<Chromosome>(std::lround(frac * static_cast<double>(GENE_MAX)));
}

double loadShared(const SharedGains& shared, Param p) {
    switch (p) {
        case Param::KP: return shared.kp.load();
        case Param::KI: return shared.ki.load();
        case Param::KD: return shared.kd.load();
    }
    return 0.0;  // unreachable
}

void storeShared(SharedGains& shared, Param p, double value) {
    switch (p) {
        case Param::KP: shared.kp.store(value); return;
        case Param::KI: shared.ki.store(value); return;
        case Param::KD: shared.kd.store(value); return;
    }
}

// ##### fitness() #####
// Goal: run one short flight through navigation::Navigation (which
// degenerates to a single NavigationStep call chain for a target this
// close) and score it by closest approach -- see this function's own
// declaration comment in the header for why 1/(dist+1.0).
double fitness(Param which, double value, const SharedGains& shared, const RocketParams& params,
               const SimulationConfig& config, const std::vector<MassComponent>& mass_components,
               const FlightData& flight_data) {
    double kp = (which == Param::KP) ? value : loadShared(shared, Param::KP);
    double ki = (which == Param::KI) ? value : loadShared(shared, Param::KI);
    double kd = (which == Param::KD) ? value : loadShared(shared, Param::KD);

    RocketKinematics sim(params, config, mass_components);
    navigation::Navigation nav(LOCAL_HOP_TARGET, config.dt, kp, ki, kd);
    auto states = sim.simulate(LOCAL_HOP_DURATION_S, flight_data, {}, &nav);

    double closest = std::numeric_limits<double>::max();
    for (const auto& state : states) {
        closest = std::min(closest, (state.position - LOCAL_HOP_TARGET).norm());
    }
    return 1.0 / (closest + FITNESS_DISTANCE_OFFSET_M);
}

}  // namespace tuning

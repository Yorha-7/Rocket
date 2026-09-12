#pragma once

#include "sim/rocket_kinematics.hpp"
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

// ##### tuning::pid_ga_tuner, the big picture #####
// Goal: an offline, three-way genetic algorithm search for a better
// NavigationStep (Kp, Ki, Kd) triple than today's grid-searched
// defaults. One GA per gain, each running on its own thread and
// evolving ONE 8-bit-encoded parameter; the three coordinate with each
// other through a small shared, atomic "current best" triple -- while
// the Kp-GA is evolving Kp, it evaluates fitness using whatever the
// Ki-GA/Kd-GA have found best SO FAR (possibly a generation or so
// stale) instead of a frozen baseline, so the three concurrent searches
// genuinely inform each other rather than running in isolation.

namespace tuning {

enum class Param { KP, KI, KD };

// Goal: one place for each gain's search range and today's known-good
// starting point -- kept decoupled from NavigationStep's own default
// constructor arguments (include/gnc/navigation.hpp) rather than
// shared, same pattern this project already uses elsewhere (e.g.
// ThrustVectorControl's own MAX_GIMBAL_DEG copy) -- just keep the two
// in sync by hand if NavigationStep's defaults ever change.
struct ParamSpec {
    double min, max, baseline;
};
ParamSpec specFor(Param p);
std::string paramName(Param p);

// ##### 8-bit binary encoding #####
// Goal: "the sample space is quite limited" -- one byte (256 discrete
// levels) per gene is plenty of resolution for a PID gain, and keeps
// every genetic operator (crossover, mutation) a handful of bit
// operations on a plain uint8_t instead of a bitset/BCD abstraction.
using Chromosome = uint8_t;
constexpr int GENE_BITS = 8;

double decode(Chromosome c, const ParamSpec& spec);
Chromosome encode(double value, const ParamSpec& spec);

// ##### SharedGains #####
// Goal: the live "current best" triple all three GA threads read from
// and write to -- how the three otherwise-independent per-parameter
// searches coordinate with each other. A reader can see a value that's
// a generation or so stale relative to a concurrent writer; that's an
// accepted characteristic of running three asynchronous coordinate-wise
// searches together, not a bug -- exact lockstep ordering doesn't
// matter for a search heuristic like this one.
struct SharedGains {
    std::atomic<double> kp{1.0};
    std::atomic<double> ki{0.6};
    std::atomic<double> kd{0.0};
};
double loadShared(const SharedGains& shared, Param p);
void storeShared(SharedGains& shared, Param p, double value);

// ##### fitness() #####
// Goal: score one candidate value for the parameter being searched,
// holding the other two at whatever SharedGains currently reports.
// Builds a short, close, single-waypoint flight (so Navigation
// collapses to exactly NavigationStep with no path-planning
// intervention mixed in -- this tunes the per-tick controller itself)
// and scores 1/(closest_approach + 1.0) -- the +1.0 keeps the score
// finite and readable (roughly 0..1) instead of blowing up as
// closest_approach -> 0.
double fitness(Param which, double value, const SharedGains& shared, const RocketParams& params,
               const SimulationConfig& config, const std::vector<MassComponent>& mass_components,
               const FlightData& flight_data);

// ##### runAllThreeGAs() #####
// Goal: the one entry point main() calls -- spins up all three
// per-parameter GAs as concurrent threads sharing one SharedGains,
// prints live progress (generation, candidate count, ETA, best
// candidate) from each, joins them, and reports the final combined
// (Kp, Ki, Kd) against today's baseline.
void runAllThreeGAs(const RocketParams& params, const SimulationConfig& config,
                     const std::vector<MassComponent>& mass_components, const FlightData& flight_data);

}  // namespace tuning

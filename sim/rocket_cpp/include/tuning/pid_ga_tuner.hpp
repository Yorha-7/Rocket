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

// Search bounds and historic comparison baseline (independent of flight defaults).
constexpr double KP_MIN = 0.0, KP_MAX = 3.0, KP_BASELINE = 1.0;
constexpr double KI_MIN = 0.0, KI_MAX = 2.0, KI_BASELINE = 0.6;
constexpr double KD_MIN = 0.0, KD_MAX = 0.5, KD_BASELINE = 0.0;

// Goal: a single fixed "local hop" target, close enough (magnitude <
// this project's own Navigation::WAYPOINT_STEP_M, 50m) that Navigation
// always collapses it to exactly one waypoint -- i.e. this evaluates
// NavigationStep's own steering law directly, uncomplicated by the
// path planner ever advancing mid-flight.
inline const Eigen::Vector3d LOCAL_HOP_TARGET(0, 20, 45);
constexpr double LOCAL_HOP_DURATION_S = 30.0;  // generous -- ground termination ends a real flight sooner

constexpr int POPULATION_SIZE = 15;
constexpr int N_GENERATIONS = 8;
constexpr double ELITE_FRACTION = 0.25;  // "fittest slice" the next parent's median comes from

constexpr double TIME_STEP_S = 0.001;
constexpr double INITIAL_TILT_DEG = 0.0;
constexpr double INITIAL_YAW_DEG = 0.0;
constexpr double FITNESS_DISTANCE_OFFSET_M = 1.0;

enum class Param { KP, KI, KD };

// Search bounds and baseline are declared above; specFor selects one gain.
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
constexpr unsigned GENE_MAX = (1u << GENE_BITS) - 1;
constexpr double MIN_PROGRESS_DENOMINATOR = 1e-6;
constexpr unsigned PARALLEL_GAIN_SEARCHES = 3;
constexpr size_t PROGRESS_REPORTS_PER_GENERATION = 5;

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
    std::atomic<double> kp{KP_BASELINE};
    std::atomic<double> ki{KI_BASELINE};
    std::atomic<double> kd{KD_BASELINE};
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

#include "tuning/pid_ga_tuner.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <random>
#include <thread>

// ##### tuning::pid_ga_runner, the big picture #####
// Goal: the actual genetic algorithm -- generating every mating/
// mutation combination the population allows (not one random pick),
// evaluating all of it in parallel, and collapsing each generation's
// swarm back down via the median of its fittest slice (see this
// project's own design discussion for why not plain top-fitness
// elitism). One instance of this whole process runs per parameter, on
// its own thread, from runAllThreeGAs() at the bottom of this file.

namespace tuning {
namespace {

constexpr int POPULATION_SIZE = 15;
constexpr int N_GENERATIONS = 8;
constexpr double ELITE_FRACTION = 0.25;  // "fittest slice" the next parent's median comes from

struct Candidate {
    Chromosome chromosome;
    double value;
    double fitness;
};

// ##### generateSwarm() #####
// Goal: "all possible mating patterns" -- every pair of parents, every
// single-point crossover position, both crossover directions -- times
// "all possible mutations" -- the child itself plus every single-bit
// flip of it. Nothing here is chosen at random and thrown away; this
// IS the whole combinatorial set, which is exactly why population size
// and generation count both stay small (see the class comment above).
std::vector<Chromosome> generateSwarm(const std::vector<Chromosome>& population) {
    std::vector<Chromosome> swarm;
    for (size_t i = 0; i < population.size(); ++i) {
        for (size_t j = i + 1; j < population.size(); ++j) {
            Chromosome a = population[i], b = population[j];
            for (int point = 1; point < GENE_BITS; ++point) {
                uint8_t high_mask = static_cast<uint8_t>(0xFFu << (GENE_BITS - point));
                uint8_t low_mask = static_cast<uint8_t>(~high_mask);
                Chromosome child_ab = static_cast<Chromosome>((a & high_mask) | (b & low_mask));
                Chromosome child_ba = static_cast<Chromosome>((b & high_mask) | (a & low_mask));
                for (Chromosome child : {child_ab, child_ba}) {
                    swarm.push_back(child);  // unmutated
                    for (int bit = 0; bit < GENE_BITS; ++bit) {
                        swarm.push_back(static_cast<Chromosome>(child ^ (1u << bit)));
                    }
                }
            }
        }
    }
    return swarm;
}

// ##### GenStats #####
// Goal: what one generation produced -- the single fittest candidate
// (for reporting only) and the median-of-fittest-slice value that
// actually becomes the next generation's seed parent.
struct GenStats {
    double best_value = 0.0;
    double best_fitness = 0.0;
    double median_value = 0.0;
    size_t swarm_size = 0;
};

// ##### runOneGeneration() #####
// Goal: evaluate every candidate in this generation's swarm across
// num_workers threads (a simple work-stealing index counter, no
// external thread-pool dependency), then reduce it down via fitness-
// sort + median-of-top-fraction. progress_cb fires periodically from
// whichever worker thread happens to cross a reporting boundary --
// caller is responsible for making it thread-safe (see runParamGA()).
GenStats runOneGeneration(Param which, const std::vector<Chromosome>& population, SharedGains& shared,
                          const RocketParams& params, const SimulationConfig& config,
                          const std::vector<MassComponent>& mass_components, const FlightData& flight_data,
                          int num_workers, const std::function<void(size_t, size_t)>& progress_cb) {
    std::vector<Chromosome> swarm = generateSwarm(population);
    std::vector<Candidate> results(swarm.size());
    std::atomic<size_t> next_index{0};
    std::atomic<size_t> completed{0};
    ParamSpec spec = specFor(which);

    auto worker = [&]() {
        size_t idx;
        while ((idx = next_index.fetch_add(1)) < swarm.size()) {
            double value = decode(swarm[idx], spec);
            double f = fitness(which, value, shared, params, config, mass_components, flight_data);
            results[idx] = {swarm[idx], value, f};
            size_t done = ++completed;
            if (progress_cb) progress_cb(done, swarm.size());
        }
    };

    std::vector<std::thread> workers;
    for (int i = 0; i < num_workers; ++i) workers.emplace_back(worker);
    for (auto& w : workers) w.join();

    std::sort(results.begin(), results.end(),
              [](const Candidate& a, const Candidate& b) { return a.fitness > b.fitness; });

    size_t slice_size = std::max<size_t>(1, results.size() * ELITE_FRACTION);
    std::vector<double> slice_values;
    slice_values.reserve(slice_size);
    for (size_t i = 0; i < slice_size; ++i) slice_values.push_back(results[i].value);
    std::sort(slice_values.begin(), slice_values.end());

    GenStats stats;
    stats.best_value = results.front().value;
    stats.best_fitness = results.front().fitness;
    stats.median_value = slice_values[slice_values.size() / 2];
    stats.swarm_size = swarm.size();
    return stats;
}

// ##### runParamGA() #####
// Goal: one parameter's whole GA -- seed with today's known-good
// constant plus random fill ("take current constants as the initial
// parents"), run N_GENERATIONS generations, print live progress under
// print_mutex (shared across all three threads so their output lines
// don't interleave mid-line), and push this parameter's improving value
// into SharedGains after every generation so the other two GAs can use
// it. Returns the final generation's representative value.
double runParamGA(Param which, SharedGains& shared, const RocketParams& params, const SimulationConfig& config,
                   const std::vector<MassComponent>& mass_components, const FlightData& flight_data,
                   std::mutex& print_mutex) {
    ParamSpec spec = specFor(which);
    std::string name = paramName(which);
    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> byte_dist(0, 255);

    std::vector<Chromosome> population;
    population.push_back(encode(spec.baseline, spec));
    for (int i = 1; i < POPULATION_SIZE; ++i) population.push_back(static_cast<Chromosome>(byte_dist(rng)));

    int num_workers = std::max(1u, std::thread::hardware_concurrency() / 3);
    double current_value = spec.baseline;

    for (int gen = 1; gen <= N_GENERATIONS; ++gen) {
        auto gen_start = std::chrono::steady_clock::now();

        auto progress_cb = [&](size_t done, size_t total) {
            size_t report_every = std::max<size_t>(1, total / 5);
            if (done % report_every != 0 && done != total) return;
            std::lock_guard<std::mutex> lock(print_mutex);
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - gen_start).count();
            double rate = done / std::max(elapsed, 1e-6);
            double eta = (total - done) / std::max(rate, 1e-6);
            std::cout << "[" << name << "] Gen " << gen << "/" << N_GENERATIONS << ": " << done << "/" << total
                      << " candidates (" << (100 * done / total) << "%) elapsed=" << elapsed
                      << "s ETA=" << eta << "s\n";
        };

        GenStats stats = runOneGeneration(which, population, shared, params, config, mass_components,
                                           flight_data, num_workers, progress_cb);
        current_value = stats.median_value;
        storeShared(shared, which, current_value);

        {
            std::lock_guard<std::mutex> lock(print_mutex);
            std::cout << "[" << name << "] Gen " << gen << "/" << N_GENERATIONS << " done ("
                      << stats.swarm_size << " candidates) -- best this gen: " << name << "="
                      << stats.best_value << " (fitness=" << stats.best_fitness << ") | next parent (median of "
                      << "fittest " << static_cast<int>(ELITE_FRACTION * 100) << "%) = " << current_value << "\n";
        }

        population.clear();
        population.push_back(encode(current_value, spec));
        for (int i = 1; i < POPULATION_SIZE; ++i) population.push_back(static_cast<Chromosome>(byte_dist(rng)));
    }

    return current_value;
}

}  // namespace

// ##### runAllThreeGAs() #####
// Goal: launch all three per-parameter GAs concurrently (one std::
// thread each, sharing one SharedGains -- see the header's own comment
// for why coordinating through a live shared value, not a frozen
// baseline, was the chosen design), then report the final triple
// against today's baseline using one consolidated fitness call.
void runAllThreeGAs(const RocketParams& params, const SimulationConfig& config,
                     const std::vector<MassComponent>& mass_components, const FlightData& flight_data) {
    std::cout << std::fixed << std::setprecision(4);

    size_t pairs = static_cast<size_t>(POPULATION_SIZE) * (POPULATION_SIZE - 1) / 2;
    size_t swarm_per_gen = pairs * 2 * (GENE_BITS - 1) * (GENE_BITS + 1);
    std::cout << "Each generation evaluates " << swarm_per_gen << " candidates per parameter ("
              << (swarm_per_gen * N_GENERATIONS) << " total per parameter, "
              << (swarm_per_gen * N_GENERATIONS * 3) << " across all three, run concurrently).\n\n";

    SharedGains shared;  // starts at NavigationStep's own defaults (see header)
    double baseline_fitness = fitness(Param::KP, shared.kp.load(), shared, params, config, mass_components, flight_data);
    std::cout << "Baseline (Kp=" << shared.kp.load() << ", Ki=" << shared.ki.load() << ", Kd=" << shared.kd.load()
              << "): fitness=" << baseline_fitness << std::endl;

    std::mutex print_mutex;
    double result_kp = 1.0, result_ki = 0.6, result_kd = 0.0;
    std::thread t_kp([&]() { result_kp = runParamGA(Param::KP, shared, params, config, mass_components, flight_data, print_mutex); });
    std::thread t_ki([&]() { result_ki = runParamGA(Param::KI, shared, params, config, mass_components, flight_data, print_mutex); });
    std::thread t_kd([&]() { result_kd = runParamGA(Param::KD, shared, params, config, mass_components, flight_data, print_mutex); });
    t_kp.join();
    t_ki.join();
    t_kd.join();

    SharedGains final_shared;
    final_shared.kp.store(result_kp);
    final_shared.ki.store(result_ki);
    final_shared.kd.store(result_kd);
    double final_fitness = fitness(Param::KP, result_kp, final_shared, params, config, mass_components, flight_data);

    std::cout << "\n==================================================\n";
    std::cout << "Final gains: Kp=" << result_kp << ", Ki=" << result_ki << ", Kd=" << result_kd
              << " (fitness=" << final_fitness << ", baseline was " << baseline_fitness << ")\n";
    std::cout << "Paste these into NavigationStep's default gains "
                 "(include/gnc/navigation.hpp) if they beat the baseline.\n";
}

}  // namespace tuning

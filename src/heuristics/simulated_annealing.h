#ifndef DIMACS2021_SIMULATED_ANNEALING_H
#define DIMACS2021_SIMULATED_ANNEALING_H

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "../cvrp.h"
#include "../moves.h"
#include "constructive.h"


namespace orcs {

    /**
     * Perform a search for a solution through Simulated Annealing heuristic.
     * @param problem Instance of the problem.
     * @param start A starting solution.
     * @param moves A vector of pointers to moves.
     * @param generator Pseudo-random number generator.
     * @param timer A timer object.
     * @param time_limit Runtime limit in seconds (default: 1E100).
     * @param verbose Verbosity (default: false).
     * @param alpha (default: 0.97).
     * @param t0 (default: 1.0).
     * @param sa_max (default: 1176628).
     * @return The best solution found.
     */
    template <typename cost_t, typename TRandom>
    Solution<cost_t> simulated_annealing(const Problem<cost_t>& problem, const Solution<cost_t>& start,
            std::vector<Move*>& moves, TRandom& generator, const cxxtimer::Timer& timer, double time_limit = 1E100,
            bool verbose = false, double alpha = 0.97, double t0 = 1.0, unsigned long long sa_max = 1176628L);

}


/*
 * Implementation
 */

template <typename cost_t, typename TRandom>
orcs::Solution<cost_t> orcs::simulated_annealing(const Problem<cost_t>& problem, const Solution<cost_t>& start,
        std::vector<Move*>& moves, TRandom& generator, const cxxtimer::Timer& timer, double time_limit, bool verbose,
        double alpha, double t0, unsigned long long sa_max) {

    // Some constants
    constexpr double EPS = 1e-6;

    // Set current and best solutions
    Solution<cost_t> best_solution = start;
    Solution<cost_t> current_solution = best_solution;

    if (verbose) {
        std::cout << best_solution << std::endl;
        //std::cout << std::fixed << std::setprecision(6) << best_solution.cost() << " (" << (problem.is_feasible(best_solution) ? "feasible" : "infeasible") << ")" << std::endl;
        std::cout.flush();
    }

    // Set parameters
    double temperature = t0;
    unsigned long long iters_in_temperature = 0;

    while (timer.count<std::chrono::milliseconds>() / 1000.0 < time_limit) {

        // Select a move
        Move* move  = nullptr;
        bool intensive_search = false;

        do {
            intensive_search = ((generator() / (double) generator.max()) < 0.5);
            move = moves[generator() % moves.size()];
        } while (!move->has_move(&current_solution, intensive_search));

        // Do move
        cost_t delta = std::any_cast<cost_t>(move->try_move(&current_solution, intensive_search));

        // if solution is improved...
        if (delta < 0) {
            move->accept();
            if (current_solution.cost() < best_solution.cost()) {
                best_solution = current_solution;
                if (verbose) {
                    std::cout << best_solution << std::endl;
                    //std::cout << std::fixed << std::setprecision(6) << best_solution.cost() << " (" << (problem.is_feasible(best_solution) ? "feasible" : "infeasible") << ")" << std::endl;
                    std::cout.flush();
                }
            }
        }

        // if solution is not improved, but is accepted...
        else if (std::abs(delta) <= EPS) {
            move->accept();
        }

        // solution is not improved, but may be accepted with a probability...
        else {
            double x = generator() / (double) generator.max();
            if (x < 1 / std::exp(delta / temperature)) {
                move->accept();
            }

            // if solution is rejected...
            else {
                move->reject();
            }
        }

        // Updates temperature
        ++iters_in_temperature;
        if (iters_in_temperature >= sa_max) {
            iters_in_temperature = 0;
            temperature = alpha * temperature;
            if (temperature < EPS) {
                temperature = t0;
            }
        }

    }

    return best_solution;
}


#endif

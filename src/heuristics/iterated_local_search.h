#ifndef DIMACS2021_ITERATED_LOCAL_SEARCH_H
#define DIMACS2021_ITERATED_LOCAL_SEARCH_H

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "../cvrp.h"
#include "../moves.h"
#include "constructive.h"
#include "variable_neighborhood_search.h"


namespace orcs {

    /**
     * Perform a search for a solution through Iterated Local Search heuristic.
     * @param problem Instance of the problem.
     * @param start A starting solution.
     * @param moves A vector of pointers to moves.
     * @param generator Pseudo-random number generator.
     * @param timer A timer object.
     * @param time_limit Runtime limit in seconds (default: 1E100).
     * @param verbose Verbosity (default: false).
     * @return The best solution found.
     */
    template <typename cost_t, typename TRandom>
    Solution<cost_t> iterated_local_search(const Problem<cost_t>& problem, const Solution<cost_t>& start,
            std::vector<Move*> moves, TRandom& generator, const cxxtimer::Timer& timer, double time_limit = 1E100,
            bool verbose = false);

}


/*
 * Implementation
 */

template <typename cost_t, typename TRandom>
orcs::Solution<cost_t> orcs::iterated_local_search(const Problem<cost_t>& problem, const Solution<cost_t>& start,
        std::vector<Move*> moves, TRandom& generator, const cxxtimer::Timer& timer, double time_limit, bool verbose) {

    // Some constants
    constexpr double EPS = 1e-6;

    // Set best solution as the start one
    Solution<cost_t> best_solution = start;
    if (verbose) {
        std::cout << best_solution << std::endl;
        std::cout.flush();
    }

    // Perform local search
    variable_neighborhood_search(best_solution, moves, timer, time_limit);
    if (verbose) {
        std::cout << best_solution << std::endl;
        std::cout.flush();
    }

    // Iterative process
    auto current_solution = best_solution;
    while (timer.count<std::chrono::milliseconds>() / 1000.0 < time_limit) {

        // Perturb the current best solution
        current_solution = best_solution;
        Move* move = moves[generator() % moves.size()];
        move->try_move(&current_solution, false);
        move->accept();

        // Local search
        variable_neighborhood_search(current_solution, moves, timer, time_limit);

        // If solution is improved...
        if (current_solution.cost() < best_solution.cost() - EPS) {
            best_solution = current_solution;
            if (verbose) {
                std::cout << best_solution << std::endl;
                std::cout.flush();
            }
        }

    }

    return best_solution;
}


#endif

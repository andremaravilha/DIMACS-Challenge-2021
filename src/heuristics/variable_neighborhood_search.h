#ifndef DIMACS2021_VARIABLE_NEIGHBORHOOD_SEARCH_H
#define DIMACS2021_VARIABLE_NEIGHBORHOOD_SEARCH_H

#include <vector>

#include "../cvrp.h"
#include "../moves.h"


namespace orcs {

    /**
     * Perform a local search based on Variable Neighborhood Search strategy.
     * @param solution Starting solution to perform the search.
     * @param moves A vector of pointers to moves.
     * @param time_limit Runtime limit in seconds (default: 1E100).
     * @param timer A timer object.
     */
    template <typename cost_t>
    void variable_neighborhood_search(Solution<cost_t>& solution, std::vector<Move*>& moves,
            const cxxtimer::Timer& timer, double time_limit = 1E100);

}


/*
 * Implementation
 */

template <typename cost_t>
void orcs::variable_neighborhood_search(Solution<cost_t>& solution, std::vector<Move*>& moves,
        const cxxtimer::Timer& timer, double time_limit) {

    constexpr double EPS = 1e-6;
    int move_idx = 0;
    Move* move = nullptr;

    while (move_idx < moves.size() && (timer.count<std::chrono::milliseconds>() / 1000.0) < time_limit) {
        move = moves[move_idx];
        cost_t delta = std::any_cast<cost_t>(move->try_move(&solution, true));
        if (delta < -EPS) {
            move->accept();
            move_idx = 0;
        } else {
            move->reject();
            ++move_idx;
        }
    }

}


#endif

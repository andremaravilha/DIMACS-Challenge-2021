#ifndef DIMACS2021_3OPT_H
#define DIMACS2021_3OPT_H

#include <any>
#include <cmath>
#include <limits>
#include <cassert>
#include <utility>
#include <algorithm>
#include <iostream>

#include "move.h"
#include "../cvrp.h"


namespace orcs {

    /**
     * A 3-opt move is an intra-route movement that consists in removing three arcs, which results in 3 sub-tours. Next,
     * these subtours are reconnect using at least two different arcs from those previously removed. Then, for each
     * triple of arcs removed, there are seven different ways of reconnecting the subtours. Considering an array
     * [0, A, B, C, D, E] as a route starting and ending at the depot (node 0), three subtours X = [0, A], Y = [B, C]
     * and Z = [D, E] are obtained after removing arcs (A,B), (C,D) and (E,0). Besides, considering R = [k, l, m, n] as
     * a subtour, the notation R' represents the reverse order of the subtour, i.e., R' = [n, m, l, k]. The seven
     * possible results of 3-opt move are:
     * X(YZ)', equivalent to a single 2-opt move (strategy 0)
     * XY'Z, equivalent to a single 2-opt move (strategy 1)
     * XYZ', equivalent to a single 2-opt move (strategy 2)
     * X(Y'Z)', equivalent to two consecutive 2-opt moves (strategy 3)
     * X(YZ')', equivalent to two consecutive 2-opt moves (strategy 4)
     * XY'Z', equivalent to two consecutive 2-opt moves (strategy 5)
     * X(Y'Z')', equivalent to three consecutive 2-opt moves (strategy 6)
     */
    template <typename cost_t, typename TRandom>
    class ThreeOpt : public Move {

    public:

        /**
         * Constructor
         * @param problem Problem instance.
         * @param generator Random number generator.
         */
        ThreeOpt(const Problem<cost_t>& problem, TRandom& generator);

        /**
         * Return the name of this movement.
         * @return The name of this movement.
         */
        const char* const name() const override;

        /**
         * Perform the move and returns the difference in the cost of the solution after the move. Note that this
         * instance of the move will be in an intermediate state until the movement is confirmed (calling to accept()
         * method) or reject (calling to reject() method).
         * @param solution_ptr A pointer to a solution.
         * @param intensive_search Set it to true if a intensive search must be performed, or set it to false to
         * perform a random move.
         * @return The difference in the cost of the solution after the move.
         */
        std::any try_move(void* solution_ptr, bool intensive_search) override;

        /**
         * Perform the best move and returns the difference in the cost of the solution after the move. Note that this
         * instance of the move will be in an intermediate state until the movement is confirmed (calling to accept()
         * method) or reject (calling to reject() method).
         * @param solution_ptr A pointer to a solution.
         * @return The difference in the cost of the solution after the move.
         */
        std::any try_best_move(void* solution_ptr) override;

        /**
         * Check if a solution satisfy the minimum requirements to try this move.
         * @param solution A pointer to a solution.
         * @return True if the solution satisfy the minimum requirements to try this move, false otherwise.
         */
        bool has_move(void* solution_ptr, bool intensive_search) override;

        /**
         * Accept the move.
         */
        void accept() override;

        /**
         * Reject the move.
         */
        void reject() override;

    private:

        int calculate_delta(int k, int idx_1, int idx_2, int idx_3, cost_t* delta) const;

        // Attributes
        constexpr static int N_RECONNECTION_STRATEGIES = 7;
        constexpr static const char* name_ = "3-opt";
        const Problem<cost_t>& problem_;
        Solution<cost_t>* solution_;
        TRandom& generator_;
        bool intermediate_state_;
        int k_, idx_1_, idx_2_, idx_3_, strategy_;
        cost_t delta_strategies_[7];
        cost_t old_cost_;
        cost_t delta_;

    };

}


/*
 * Implementation.
 */

template <typename cost_t, typename TRandom>
orcs::ThreeOpt<cost_t, TRandom>::ThreeOpt(const orcs::Problem<cost_t>& problem, TRandom& generator) :
problem_(problem), generator_(generator), intermediate_state_(false), solution_(nullptr)
{
    // Do nothing here.
}


template <typename cost_t, typename TRandom>
const char* const orcs::ThreeOpt<cost_t, TRandom>::name() const {
    return name_;
}

template <typename cost_t, typename TRandom>
std::any orcs::ThreeOpt<cost_t, TRandom>::try_move(void* solution_ptr, bool intensive_search) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));
    assert(("Solution has no possible movements.", has_move((Solution<cost_t>*) solution_ptr, intensive_search)));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Randomly selects a route and three arcs to remove from it
    do {
        k_ = generator_() % solution_->count_routes();
    } while (solution_->count_nodes(k_) < 5);

    idx_1_ = (generator_() % (solution_->count_nodes(k_) + 1));

    do {
        idx_2_ = (generator_() % solution_->count_nodes(k_)) + 1;
    } while (idx_2_ == idx_1_);

    if (idx_2_ < idx_1_) std::swap(idx_1_, idx_2_);

    do {
        idx_3_ = (generator_() % solution_->count_nodes(k_)) + 1;
    } while (idx_3_ == idx_1_ || idx_3_ == idx_2_);

    if (idx_3_ < idx_2_) std::swap(idx_2_, idx_3_);
    if (idx_2_ < idx_1_) std::swap(idx_1_, idx_2_);

    // Calculate delta cost for each reconnection strategy and find the best one
    strategy_ = calculate_delta(k_, idx_1_, idx_2_, idx_3_, delta_strategies_);

    // If intensive search is not enabled,
    // randomly chooses a reconnection strategy
    if (!intensive_search) {
        strategy_ = generator_() % N_RECONNECTION_STRATEGIES;
    }

    delta_ = delta_strategies_[strategy_];
    return delta_;
}


template <typename cost_t, typename TRandom>
std::any orcs::ThreeOpt<cost_t, TRandom>::try_best_move(void* solution_ptr) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Find best move
    delta_ = std::numeric_limits<cost_t>::max();
    for (int k = 0; k < solution_->count_routes(); ++k) {
        for (int idx_1 = 1; idx_1 <= solution_->count_nodes(k) - 2; ++idx_1) {
            for (int idx_2 = idx_1 + 1; idx_2 <= solution_->count_nodes(k) - 1; ++idx_2) {
                for (int idx_3 = idx_2 + 1; idx_3 <= solution_->count_nodes(k); ++idx_3) {
                    int strategy = calculate_delta(k, idx_1, idx_2, idx_3, delta_strategies_);
                    if (delta_strategies_[strategy] < delta_) {
                        delta_ = delta_strategies_[strategy];
                        strategy_ = strategy;
                        k_ = k;
                        idx_1_ = idx_1;
                        idx_2_ = idx_2;
                        idx_3_ = idx_3;
                    }
                }
            }
        }
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
bool orcs::ThreeOpt<cost_t, TRandom>::has_move(void* solution_ptr, bool intensive_search) {
    assert(("Checking the existence of moves in an intermediate state.", intermediate_state_ == false));

    Solution<cost_t>* solution = (Solution<cost_t>*) solution_ptr;
    for (int k = 0; k < solution->count_routes(); ++k) {
        if (solution->count_nodes(k) >= 5) {
            return true;
        }
    }

    return false;
}


template <typename cost_t, typename TRandom>
void orcs::ThreeOpt<cost_t, TRandom>::accept() {
    assert(("Trying to accept a move in a non-intermediate state.", intermediate_state_ == true));

    switch (strategy_) {

        // Strategy 0: X(YZ)'
        case 0:
            solution_->reverse_route(k_, idx_1_ + 1, idx_3_);
            break;

        // Strategy 1: XY'Z
        case 1:
            solution_->reverse_route(k_, idx_1_ + 1, idx_2_);
            break;

        // Strategy 2: XYZ'
        case 2:
            solution_->reverse_route(k_, idx_2_ + 1, idx_3_);
            break;

        // Strategy 3: X(Y'Z)'
        case 3:
            solution_->reverse_route(k_, idx_1_ + 1, idx_2_);
            solution_->reverse_route(k_, idx_1_ + 1, idx_3_);
            break;

        // Strategy 4: X(YZ')'
        case 4:
            solution_->reverse_route(k_, idx_2_ + 1, idx_3_);
            solution_->reverse_route(k_, idx_1_ + 1, idx_3_);
            break;

        // Strategy 5: XY'Z'
        case 5:
            solution_->reverse_route(k_, idx_1_ + 1, idx_2_);
            solution_->reverse_route(k_, idx_2_ + 1, idx_3_);
            break;

        // Strategy 6: X(Y'Z')'
        case 6:
            solution_->reverse_route(k_, idx_1_ + 1, idx_2_);
            solution_->reverse_route(k_, idx_2_ + 1, idx_3_);
            solution_->reverse_route(k_, idx_1_ + 1, idx_3_);
            break;
    }

    intermediate_state_ = false;

    assert(("Unexpected cost after accepting move.", std::abs(solution_->cost() - (old_cost_ + delta_)) < 1e-6));
}


template <typename cost_t, typename TRandom>
void orcs::ThreeOpt<cost_t, TRandom>::reject() {
    assert(("Trying to reject a move in a non-intermediate state.", intermediate_state_ == true));
    solution_ = nullptr;
    intermediate_state_ = false;
}


template <typename cost_t, typename TRandom>
int orcs::ThreeOpt<cost_t, TRandom>::calculate_delta(int k, int idx_1, int idx_2, int idx_3, cost_t* delta) const {

    // Arcs x, y and z to remove from the route
    int x1 = solution_->get_node(k, idx_1);
    int x2 = solution_->get_node(k, idx_1 + 1);
    int y1 = solution_->get_node(k, idx_2);
    int y2 = solution_->get_node(k, idx_2 + 1);
    int z1 = solution_->get_node(k, idx_3);
    int z2 = solution_->get_node(k, idx_3 + 1);

    // Cost of arcs to remove
    cost_t base = problem_.cost(x1, x2) + problem_.cost(y1, y2) + problem_.cost(z1, z2);

    // Compute the delta cost for each reconnection strategy and identified the best one
    int idx_best_strategy;

    // Strategy 0: X(YZ)'
    delta[0] = problem_.cost(x1, z1) + problem_.cost(y2, y1) + problem_.cost(x2, z2) - base;
    idx_best_strategy = 0;

    // Strategy 1: XY'Z
    delta[1] = problem_.cost(x1, y1) + problem_.cost(x2, y2) + problem_.cost(z1, z2) - base;
    if (delta[1] < delta[idx_best_strategy]) idx_best_strategy = 1;

    // Strategy 2: XYZ'
    delta[2] = problem_.cost(x1, x2) + problem_.cost(y1, z1) + problem_.cost(y2, z2) - base;
    if (delta[2] < delta[idx_best_strategy]) idx_best_strategy = 2;

    // Strategy 3: X(Y'Z)'
    delta[3] = problem_.cost(x1, z1) + problem_.cost(y2, x2) + problem_.cost(y1, z2) - base;
    if (delta[3] < delta[idx_best_strategy]) idx_best_strategy = 3;

    // Strategy 4: X(YZ')'
    delta[4] = problem_.cost(x1, y2) + problem_.cost(z1, y1) + problem_.cost(x2, z2) - base;
    if (delta[4] < delta[idx_best_strategy]) idx_best_strategy = 4;

    // Strategy 5: XY'Z'
    delta[5] = problem_.cost(x1, y1) + problem_.cost(x2, z1) + problem_.cost(y2, z2) - base;
    if (delta[5] < delta[idx_best_strategy]) idx_best_strategy = 5;

    // Strategy 6: X(Y'Z')'
    delta[6] = problem_.cost(x1, y2) + problem_.cost(z1, x2) + problem_.cost(y1, z2) - base;
    if (delta[6] < delta[idx_best_strategy]) idx_best_strategy = 6;

    return idx_best_strategy;
}


#endif

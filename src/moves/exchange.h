#ifndef DIMACS2021_EXCHANGE_H
#define DIMACS2021_EXCHANGE_H

#include <any>
#include <limits>
#include <cassert>
#include <utility>
#include <iostream>

#include "move.h"
#include "../cvrp.h"


namespace orcs {

    /**
     * Exchange move is an intra-route movement that consists in exchanging two customers in the same route.
     */
    template <typename cost_t, typename TRandom>
    class Exchange : public Move {

    public:

        /**
         * Constructor
         * @param problem Problem instance.
         * @param generator Random number generator.
         */
        Exchange(const Problem<cost_t>& problem, TRandom& generator);

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

        cost_t calculate_delta(int k, int idx_1, int idx_2) const;

        // Attributes
        constexpr static const char* name_ = "Exchange";
        const Problem<cost_t>& problem_;
        Solution<cost_t>* solution_;
        TRandom& generator_;
        bool intermediate_state_;
        int k_;
        int idx_1_, idx_2_;
        cost_t old_cost_;
        cost_t delta_;

    };

}


/*
 * Implementation.
 */

template <typename cost_t, typename TRandom>
orcs::Exchange<cost_t, TRandom>::Exchange(const orcs::Problem<cost_t>& problem, TRandom& generator) :
problem_(problem), generator_(generator), intermediate_state_(false), solution_(nullptr)
{
    // Do nothing here.
}


template <typename cost_t, typename TRandom>
const char* const orcs::Exchange<cost_t, TRandom>::name() const {
    return name_;
}

template <typename cost_t, typename TRandom>
std::any orcs::Exchange<cost_t, TRandom>::try_move(void* solution_ptr, bool intensive_search) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));
    assert(("Solution has no possible movements.", has_move((Solution<cost_t>*) solution_ptr, intensive_search)));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // intensive search (but not exhaustive)
    if (intensive_search) {

        do {
            k_ = generator_() % solution_->count_routes();
        } while (solution_->count_nodes(k_) < 2);

        idx_1_ = (generator_() % solution_->count_nodes(k_)) + 1;

        delta_ = std::numeric_limits<cost_t>::max();
        for (int idx_2 = 1; idx_2 <= solution_->count_nodes(k_); ++idx_2) {
            if (idx_2 != idx_1_) {
                cost_t delta = calculate_delta(k_, idx_1_, idx_2);
                if (delta < delta_) {
                    delta_ = delta;
                    idx_2_ = idx_2;
                }
            }
        }

    // random move
    } else {

        do {
            k_ = generator_() % solution_->count_routes();
        } while (solution_->count_nodes(k_) < 2);

        idx_1_ = (generator_() % solution_->count_nodes(k_)) + 1;

        do {
            idx_2_ = (generator_() % solution_->count_nodes(k_)) + 1;
        } while (idx_2_ == idx_1_);

        delta_ = calculate_delta(k_, idx_1_, idx_2_);
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
std::any orcs::Exchange<cost_t, TRandom>::try_best_move(void* solution_ptr) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Find best move
    delta_ = std::numeric_limits<cost_t>::max();
    for (int k = 0; k < solution_->count_routes(); ++k) {
        for (int idx_1 = 1; idx_1 <= solution_->count_nodes(k); ++idx_1) {
            for (int idx_2 = idx_1 + 1; idx_2 <= solution_->count_nodes(k); ++idx_2) {
                cost_t delta = calculate_delta(k, idx_1, idx_2);
                if (delta < delta_) {
                    delta_ = delta;
                    k_ = k;
                    idx_1_ = idx_1;
                    idx_2_ = idx_2;
                }
            }
        }
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
bool orcs::Exchange<cost_t, TRandom>::has_move(void* solution_ptr, bool intensive_search) {
    assert(("Checking the existence of moves in an intermediate state.", intermediate_state_ == false));

    Solution<cost_t>* solution = (Solution<cost_t>*) solution_ptr;
    for (int k = 0; k < solution->count_routes(); ++k) {
        if (solution->count_nodes(k) >= 2) {
            return true;
        }
    }

    return false;
}


template <typename cost_t, typename TRandom>
void orcs::Exchange<cost_t, TRandom>::accept() {
    assert(("Trying to accept a move in a non-intermediate state.", intermediate_state_ == true));

    int node_1 = solution_->get_node(k_, idx_1_);
    int node_2 = solution_->get_node(k_, idx_2_);
    solution_->set_node(k_, idx_2_, node_1);
    solution_->set_node(k_, idx_1_, node_2);
    intermediate_state_ = false;

    assert(("Unexpected cost after accepting move.", std::abs(solution_->cost() - (old_cost_ + delta_)) < 1e-6));
}


template <typename cost_t, typename TRandom>
void orcs::Exchange<cost_t, TRandom>::reject() {
    assert(("Trying to reject a move in a non-intermediate state.", intermediate_state_ == true));
    solution_ = nullptr;
    intermediate_state_ = false;
}


template <typename cost_t, typename TRandom>
cost_t orcs::Exchange<cost_t, TRandom>::calculate_delta(int k, int idx_1, int idx_2) const {
    if (idx_1 > idx_2) {
        std::swap(idx_1, idx_2);
    }

    int node_1 = solution_->get_node(k, idx_1);
    int node_2 = solution_->get_node(k, idx_2);

    cost_t delta;
    if (std::abs(idx_1 - idx_2) > 1) {
        delta = problem_.cost(solution_->get_node(k, idx_2 - 1), node_1)
                + problem_.cost(node_1, solution_->get_node(k, idx_2 + 1))
                + problem_.cost(solution_->get_node(k, idx_1 - 1), node_2)
                + problem_.cost(node_2, solution_->get_node(k, idx_1 + 1))
                - problem_.cost(solution_->get_node(k, idx_2 - 1), node_2)
                - problem_.cost(node_2, solution_->get_node(k, idx_2 + 1))
                - problem_.cost(solution_->get_node(k, idx_1 - 1), node_1)
                - problem_.cost(node_1, solution_->get_node(k, idx_1 + 1));
    } else {
        delta = problem_.cost(solution_->get_node(k, idx_1 - 1), node_2)
                + problem_.cost(node_1, solution_->get_node(k, idx_2 + 1))
                - problem_.cost(solution_->get_node(k, idx_1 - 1), node_1)
                - problem_.cost(node_2, solution_->get_node(k, idx_2 + 1));
    }

    return delta;
}


#endif

#ifndef DIMACS2021_SHIFT_1_H
#define DIMACS2021_SHIFT_1_H

#include <any>
#include <limits>
#include <cassert>

#include "move.h"
#include "../cvrp.h"


namespace orcs {

    /**
     * Shift(1,0) move is an inter-route movement that relocates a customer from one route to another route.
     */
    template <typename cost_t, typename TRandom>
    class Shift_1 : public Move {

    public:

        /**
         * Constructor
         * @param problem Problem instance.
         * @param generator Random number generator.
         */
        Shift_1(const Problem<cost_t>& problem, TRandom& generator);

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

        cost_t calculate_delta(int k_source, int idx_source, int k_target, int idx_target) const;

        // Attributes
        constexpr static const char* name_ = "Shift(1,0)";
        const Problem<cost_t>& problem_;
        Solution<cost_t>* solution_;
        TRandom& generator_;
        bool intermediate_state_;
        int k_source_, k_target_;
        int idx_source_, idx_target_;
        cost_t old_cost_;
        cost_t delta_;

    };

}


/*
 * Implementation.
 */

template <typename cost_t, typename TRandom>
orcs::Shift_1<cost_t, TRandom>::Shift_1(const orcs::Problem<cost_t>& problem, TRandom& generator) :
problem_(problem), generator_(generator), intermediate_state_(false), solution_(nullptr)
{
    // Do nothing here.
}


template <typename cost_t, typename TRandom>
const char* const orcs::Shift_1<cost_t, TRandom>::name() const {
    return name_;
}


template <typename cost_t, typename TRandom>
std::any orcs::Shift_1<cost_t, TRandom>::try_move(void* solution_ptr, bool intensive_search) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));
    assert(("Solution has no possible movements.", has_move((Solution<cost_t>*) solution_ptr, intensive_search)));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Select a source route and node, and a target route and position
    do {

        do {
            k_source_ = generator_() % solution_->count_routes();
        } while (solution_->count_nodes(k_source_) < 1);

        do {
            k_target_ = generator_() % solution_->count_routes();
        } while (k_target_ == k_source_);

        idx_source_ = (generator_() % solution_->count_nodes(k_source_)) + 1;

    } while(solution_->load(k_target_) + problem_.demand(solution_->get_node(k_source_, idx_source_)) > problem_.capacity());

    idx_target_ = (generator_() % (solution_->count_nodes(k_target_) + 1)) + 1;
    delta_ = calculate_delta(k_source_, idx_source_, k_target_, idx_target_);

    // Intensive search (but not exhaustive)
    // Find the best position in k_target to insert the node
    if (intensive_search) {
        delta_ = std::numeric_limits<cost_t>::max();
        for (int idx_target = 1; idx_target <= solution_->count_nodes(k_target_) + 1; ++idx_target) {
            cost_t delta = calculate_delta(k_source_, idx_source_, k_target_, idx_target);
            if (delta < delta_) {
                delta_ = delta;
                idx_target_ = idx_target;
            }
        }
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
std::any orcs::Shift_1<cost_t, TRandom>::try_best_move(void* solution_ptr) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Find best move
    delta_ = std::numeric_limits<cost_t>::max();
    for (int k_source = 0; k_source < solution_->count_routes(); ++k_source) {
        for (int idx_source = 1; idx_source <= solution_->count_nodes(k_source); ++idx_source) {
            int node = solution_->get_node(k_source, idx_source);
            for (int k_target = 0; k_target < solution_->count_routes(); ++k_target) {
                if (k_target != k_source && solution_->load(k_target) + problem_.demand(node) <= problem_.capacity()) {
                    for (int idx_target = 1; idx_target <= solution_->count_nodes(k_target) + 1; ++idx_target) {
                        cost_t delta = calculate_delta(k_source, idx_source, k_target, idx_target);
                        if (delta < delta_) {
                            delta_ = delta;
                            k_source_ = k_source;
                            idx_source_ = idx_source;
                            k_target_ = k_target;
                            idx_target_ = idx_target;
                        }
                    }
                }
            }
        }
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
bool orcs::Shift_1<cost_t, TRandom>::has_move(void* solution_ptr, bool intensive_search) {
    assert(("Checking the existence of moves in an intermediate state.", intermediate_state_ == false));
    Solution<cost_t>* solution = (Solution<cost_t>*) solution_ptr;
    return (solution->count_routes() >= 2);
}


template <typename cost_t, typename TRandom>
void orcs::Shift_1<cost_t, TRandom>::accept() {
    assert(("Trying to accept a move in a non-intermediate state.", intermediate_state_ == true));

    int node = solution_->get_node(k_source_, idx_source_);
    solution_->remove_node(k_source_, idx_source_);
    solution_->insert_node(k_target_, idx_target_, node);
    intermediate_state_ = false;

    assert(("Unexpected cost after accepting move.", std::abs(solution_->cost() - (old_cost_ + delta_)) < 1e-6));
}


template <typename cost_t, typename TRandom>
void orcs::Shift_1<cost_t, TRandom>::reject() {
    assert(("Trying to reject a move in a non-intermediate state.", intermediate_state_ == true));
    solution_ = nullptr;
    intermediate_state_ = false;
}


template <typename cost_t, typename TRandom>
cost_t orcs::Shift_1<cost_t, TRandom>::calculate_delta(int k_source, int idx_source, int k_target, int idx_target) const {
    int node = solution_->get_node(k_source, idx_source);
    cost_t delta = problem_.cost(solution_->get_node(k_target, idx_target - 1), node)
             + problem_.cost(node, solution_->get_node(k_target, idx_target))
             - problem_.cost(solution_->get_node(k_target, idx_target - 1), solution_->get_node(k_target, idx_target))
             - problem_.cost(solution_->get_node(k_source, idx_source - 1), node)
             - problem_.cost(node, solution_->get_node(k_source, idx_source + 1))
             + problem_.cost(solution_->get_node(k_source, idx_source - 1), solution_->get_node(k_source, idx_source + 1));

    return delta;
}


#endif

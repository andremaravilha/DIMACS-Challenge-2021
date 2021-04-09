#ifndef DIMACS2021_SWAP_1_1_H
#define DIMACS2021_SWAP_1_1_H

#include <any>
#include <limits>
#include <cassert>

#include "move.h"
#include "../cvrp.h"


namespace orcs {

    /**
     * A Swap(1,1) move is an inter-route movement that exchanges two customers from different routes.
     */
    template <typename cost_t, typename TRandom>
    class Swap_1_1 : public Move {

    public:

        /**
         * Constructor
         * @param problem Problem instance.
         * @param generator Random number generator.
         */
        Swap_1_1(const Problem<cost_t>& problem, TRandom& generator);

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

        cost_t calculate_delta(int k1, int idx_1, int k2, int idx_2) const;

        // Attributes
        constexpr static const char* name_ = "Swap(1,1)";
        const Problem<cost_t>& problem_;
        Solution<cost_t>* solution_;
        TRandom& generator_;
        bool intermediate_state_;
        int k1_, k2_;
        int idx_1_, idx_2_;
        cost_t old_cost_;
        cost_t delta_;

    };

}


/*
 * Implementation.
 */

template <typename cost_t, typename TRandom>
orcs::Swap_1_1<cost_t, TRandom>::Swap_1_1(const orcs::Problem<cost_t>& problem, TRandom& generator) :
problem_(problem), generator_(generator), intermediate_state_(false), solution_(nullptr)
{
    // Do nothing here.
}


template <typename cost_t, typename TRandom>
const char* const orcs::Swap_1_1<cost_t, TRandom>::name() const {
    return name_;
}


template <typename cost_t, typename TRandom>
std::any orcs::Swap_1_1<cost_t, TRandom>::try_move(void* solution_ptr, bool intensive_search) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));
    assert(("Solution has no possible movements.", has_move((Solution<cost_t>*) solution_ptr, intensive_search)));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    int node_1, node_2;
    do {

        do {
            k1_ = generator_() % solution_->count_routes();
        } while (solution_->count_nodes(k1_) < 1);

        idx_1_ = (generator_() % solution_->count_nodes(k1_)) + 1;
        node_1 = solution_->get_node(k1_, idx_1_);

        do {
            k2_ = generator_() % solution_->count_routes();
        } while (k2_ == k1_ || solution_->count_nodes(k2_) < 1);

        idx_2_ = (generator_() % solution_->count_nodes(k2_)) + 1;
        node_2 = solution_->get_node(k2_, idx_2_);

    } while(solution_->load(k1_) - problem_.demand(node_1) + problem_.demand(node_2) > problem_.capacity()
            || solution_->load(k2_) - problem_.demand(node_2) + problem_.demand(node_1) > problem_.capacity());

    delta_ = calculate_delta(k1_, idx_1_, k2_, idx_2_);

    // Intensive search (but not exhaustive)
    // Find the best position in k2 to insert the node_1
    if (intensive_search) {
        delta_ = std::numeric_limits<cost_t>::max();
        for (int idx_2 = 1; idx_2 <= solution_->count_nodes(k2_); ++idx_2) {
            node_2 = solution_->get_node(k2_, idx_2);
            if (solution_->load(k1_) - problem_.demand(node_1) + problem_.demand(node_2) <= problem_.capacity()
                    && solution_->load(k2_) - problem_.demand(node_2) + problem_.demand(node_1) <= problem_.capacity()) {

                cost_t delta = calculate_delta(k1_, idx_1_, k2_, idx_2);
                if (delta < delta_) {
                    delta_ = delta;
                    idx_2_ = idx_2;
                }

            }
        }
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
std::any orcs::Swap_1_1<cost_t, TRandom>::try_best_move(void* solution_ptr) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Find best move
    delta_ = std::numeric_limits<cost_t>::max();
    for (int k1 = 0; k1 < solution_->count_routes(); ++k1) {
        for (int k2 = k1 + 1; k2 < solution_->count_routes(); ++k2) {
            for (int idx_1 = 1; idx_1 <= solution_->count_nodes(k1); ++idx_1) {
                for (int idx_2 = 1; idx_2 <= solution_->count_nodes(k2); ++idx_2) {
                    int node_1 = solution_->get_node(k1, idx_1);
                    int node_2 = solution_->get_node(k2, idx_2);
                    if (solution_->load(k1) - problem_.demand(node_1) + problem_.demand(node_2) <= problem_.capacity()
                            && solution_->load(k2) - problem_.demand(node_2) + problem_.demand(node_1) <= problem_.capacity()) {

                        cost_t delta = calculate_delta(k1, idx_1, k2, idx_2);
                        if (delta < delta_) {
                            delta_ = delta;
                            k1_ = k1;
                            idx_1_ = idx_1;
                            k2_ = k2;
                            idx_2_ = idx_2;
                        }

                    }
                }
            }
        }
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
bool orcs::Swap_1_1<cost_t, TRandom>::has_move(void* solution_ptr, bool intensive_search) {
    assert(("Checking the existence of moves in an intermediate state.", intermediate_state_ == false));
    Solution<cost_t>* solution = (Solution<cost_t>*) solution_ptr;
    return solution->count_routes() >= 2;
}


template <typename cost_t, typename TRandom>
void orcs::Swap_1_1<cost_t, TRandom>::accept() {
    assert(("Trying to accept a move in a non-intermediate state.", intermediate_state_ == true));

    int node_1 = solution_->get_node(k1_, idx_1_);
    int node_2 = solution_->get_node(k2_, idx_2_);

    solution_->set_node(k1_, idx_1_, node_2);
    solution_->set_node(k2_, idx_2_, node_1);
    intermediate_state_ = false;

    assert(("Unexpected cost after accepting move.", std::abs(solution_->cost() - (old_cost_ + delta_)) < 1e-6));
}


template <typename cost_t, typename TRandom>
void orcs::Swap_1_1<cost_t, TRandom>::reject() {
    assert(("Trying to reject a move in a non-intermediate state.", intermediate_state_ == true));
    solution_ = nullptr;
    intermediate_state_ = false;
}


template <typename cost_t, typename TRandom>
cost_t orcs::Swap_1_1<cost_t, TRandom>::calculate_delta(int k1, int idx_1, int k2, int idx_2) const {
    int node_1 = solution_->get_node(k1, idx_1);
    int node_2 = solution_->get_node(k2, idx_2);
    cost_t delta = problem_.cost(solution_->get_node(k1, idx_1 - 1), node_2)
                   + problem_.cost(node_2, solution_->get_node(k1, idx_1 + 1))
                   - problem_.cost(solution_->get_node(k1, idx_1 - 1), node_1)
                   - problem_.cost(node_1, solution_->get_node(k1, idx_1 + 1))
                   + problem_.cost(solution_->get_node(k2, idx_2 - 1), node_1)
                   + problem_.cost(node_1, solution_->get_node(k2, idx_2 + 1))
                   - problem_.cost(solution_->get_node(k2, idx_2 - 1), node_2)
                   - problem_.cost(node_2, solution_->get_node(k2, idx_2 + 1));

    return delta;
}


#endif

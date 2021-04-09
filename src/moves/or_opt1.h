#ifndef DIMACS2021_OR_OPT1_H
#define DIMACS2021_OR_OPT1_H

#include <any>
#include <limits>
#include <cassert>

#include "move.h"
#include "../cvrp.h"


namespace orcs {

    /**
     * Or-Opt1 move is an intra-route movement that consists in removing one customer from a given route and reinserting
     * it into another position of the same route. The class of Or-Opt moves are proposed in [1]. This move is also
     * known as Reinsertion move.
     *
     * [1] Or, I. Traveling salesman-type combination problems and their relation to the logistics of blood banking.
     * USA: Northwestern University, PhD dissertation, 1976.
     */
    template <typename cost_t, typename TRandom>
    class OrOpt1 : public Move {

    public:

        /**
         * Constructor
         * @param problem Problem instance.
         * @param generator Random number generator.
         */
        OrOpt1(const Problem<cost_t>& problem, TRandom& generator);

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

        cost_t calculate_delta(int k, int idx_source, int idx_target) const;

        // Attributes
        constexpr static const char* name_ = "Or-Opt1";
        const Problem<cost_t>& problem_;
        Solution<cost_t>* solution_;
        TRandom& generator_;
        bool intermediate_state_;
        int k_, idx_source_, idx_target_;
        cost_t old_cost_;
        cost_t delta_;

    };

}


/*
 * Implementation.
 */

template <typename cost_t, typename TRandom>
orcs::OrOpt1<cost_t, TRandom>::OrOpt1(const orcs::Problem<cost_t>& problem, TRandom& generator) :
problem_(problem), generator_(generator), intermediate_state_(false), solution_(nullptr)
{
    // Do nothing here.
}


template <typename cost_t, typename TRandom>
const char* const orcs::OrOpt1<cost_t, TRandom>::name() const {
    return name_;
}


template <typename cost_t, typename TRandom>
std::any orcs::OrOpt1<cost_t, TRandom>::try_move(void* solution_ptr, bool intensive_search) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));
    assert(("Solution has no possible movements.", has_move((Solution<cost_t>*) solution_ptr, intensive_search)));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Intensive search (but not exhaustive), if enabled...
    if (intensive_search) {

        do {
            k_ = generator_() % solution_->count_routes();
        } while (solution_->count_nodes(k_) < 2);

        idx_source_ = (generator_() % solution_->count_nodes(k_)) + 1;

        delta_ = std::numeric_limits<cost_t>::max();
        for (int idx_target = 1; idx_target <= solution_->count_nodes(k_) + 1; ++idx_target) {
            if (idx_target != idx_source_ && idx_target != idx_source_ + 1) {
                cost_t delta = calculate_delta(k_, idx_source_, idx_target);
                if (delta < delta_) {
                    delta_ = delta;
                    idx_target_ = idx_target;
                }
            }
        }

    // Random move...
    } else {

        do {
            k_ = generator_() % solution_->count_routes();
        } while (solution_->count_nodes(k_) < 2);

        idx_source_ = (generator_() % solution_->count_nodes(k_)) + 1;

        do {
            idx_target_ = (generator_() % (solution_->count_nodes(k_) + 1)) + 1;
        } while (idx_target_ == idx_source_ || idx_target_ == idx_source_ + 1);

        delta_ = calculate_delta(k_, idx_source_, idx_target_);
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
std::any orcs::OrOpt1<cost_t, TRandom>::try_best_move(void* solution_ptr) {
    assert(("Attempt to perform move in an intermediate state.", intermediate_state_ == false));

    // Set current solution
    solution_ = (Solution<cost_t>*) solution_ptr;
    old_cost_ = solution_->cost();
    intermediate_state_ = true;

    // Find best move
    delta_ = std::numeric_limits<cost_t>::max();
    for (int k = 0; k < solution_->count_routes(); ++k) {
        for (int idx_source = 1; idx_source <= solution_->count_nodes(k); ++idx_source) {
            for (int idx_target = 1; idx_target <= solution_->count_nodes(k) + 1; ++idx_target) {
                if (idx_target != idx_source && idx_target != idx_source + 1) {

                    cost_t delta = calculate_delta(k, idx_source, idx_target);
                    if (delta < delta_) {
                        delta_ = delta;
                        k_ = k;
                        idx_source_ = idx_source;
                        idx_target_ = idx_target;
                    }

                }
            }
        }
    }

    return delta_;
}


template <typename cost_t, typename TRandom>
bool orcs::OrOpt1<cost_t, TRandom>::has_move(void* solution_ptr, bool intensive_search) {
    assert(("Checking the existence of moves in an intermediate state.", intermediate_state_ == false));
    Solution<cost_t>* solution = (Solution<cost_t>*) solution_ptr;
    return solution->count_routes() >= 1;
}


template <typename cost_t, typename TRandom>
void orcs::OrOpt1<cost_t, TRandom>::accept() {
    assert(("Trying to accept a move in a non-intermediate state.", intermediate_state_ == true));

    int node = solution_->get_node(k_, idx_source_);
    solution_->insert_node(k_, idx_target_, node);
    if (idx_target_ <= idx_source_) {
        solution_->remove_node(k_, idx_source_+ 1);
    } else {
        solution_->remove_node(k_, idx_source_);
    }

    intermediate_state_ = false;

    assert(("Unexpected cost after accepting move.", std::abs(solution_->cost() - (old_cost_ + delta_)) < 1e-6));
}


template <typename cost_t, typename TRandom>
void orcs::OrOpt1<cost_t, TRandom>::reject() {
    assert(("Trying to reject a move in a non-intermediate state.", intermediate_state_ == true));
    solution_ = nullptr;
    intermediate_state_ = false;
}


template <typename cost_t, typename TRandom>
cost_t orcs::OrOpt1<cost_t, TRandom>::calculate_delta(int k, int idx_source, int idx_target) const {
    int node = solution_->get_node(k, idx_source);
    cost_t delta = problem_.cost(solution_->get_node(k, idx_target - 1), node)
                   + problem_.cost(node, solution_->get_node(k, idx_target))
                   - problem_.cost(solution_->get_node(k, idx_target - 1), solution_->get_node(k, idx_target))
                   - problem_.cost(solution_->get_node(k, idx_source - 1), node)
                   - problem_.cost(node, solution_->get_node(k, idx_source + 1))
                   + problem_.cost(solution_->get_node(k, idx_source - 1), solution_->get_node(k, idx_source + 1));

    return delta;
}


#endif

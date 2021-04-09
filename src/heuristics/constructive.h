#ifndef DIMACS2021_CONSTRUCTIVE_H
#define DIMACS2021_CONSTRUCTIVE_H

#include <algorithm>
#include <list>
#include <limits>
#include <random>
#include <iostream>

#include "../cvrp.h"


namespace orcs {

    /**
     * Build a solution for the CVRP.
     * @param problem Instance of the problem.
     * @param generator Random number generator.
     * @return A solution to the problem.
     */
    template <typename cost_t, class TRandom>
    Solution<cost_t> random_constructive(const Problem<cost_t>& problem, TRandom& generator);

    /**
     * Build a solution for the CVRP.
     * @param problem Instance of the problem.
     * @param generator Random number generator.
     * @return A solution to the problem.
     */
    template <typename cost_t, class TRandom>
    Solution<cost_t> modified_cheapest_insertion_constructive(const Problem<cost_t>& problem, TRandom& generator);

}


/*
 * Implementation
 */

template <typename cost_t, class TRandom>
orcs::Solution<cost_t> orcs::random_constructive(const Problem<cost_t>& problem, TRandom& generator) {

    // Shuffle customer nodes
    int n_customers = problem.dimension() - 1;
    int* nodes = new int[n_customers];
    for (int i = 0; i < n_customers; ++i) {
        nodes[i] = i + 1;
    }

    std::shuffle(nodes, nodes + n_customers, generator);

    // Build a solution node by node
    Solution<cost_t> solution(problem);
    int k = solution.add_route();

    for (int i = 0; i < n_customers; ++i) {
        if (solution.load(k) + problem.demand(nodes[i]) > problem.capacity()) {
            k = solution.add_route();
        }
        solution.insert_node(k, nodes[i]);
    }

    return solution;
}


template <typename cost_t, class TRandom>
orcs::Solution<cost_t> orcs::modified_cheapest_insertion_constructive(const Problem<cost_t>& problem, TRandom& generator) {

    // Create an empty solution
    Solution<cost_t> solution(problem);
    solution.add_route();
    double gamma = 0.4;

    // Initialize the candidate list
    int n_customers = problem.dimension() - 1;
    std::list<int> candidate_list;
    for (int i = 1; i <= n_customers; ++i) {
        candidate_list.push_back(i);
    }

    // Build solution, node-by-node
    while (!candidate_list.empty()) {
//        std::cout << "Size of candidate list: " << candidate_list.size() << std::endl;
//        std::cout << "Number of routes:       " << solution.count_routes() << std::endl;
//        std::cout << "Partial cost:           " << solution.cost() << std::endl << std::endl;

        auto best_iter = candidate_list.begin();
        double best_cost = std::numeric_limits<double>::max();
        int best_k = 0;
        int best_idx = 1;

        for (auto iter = candidate_list.begin(); iter != candidate_list.end(); ++iter) {
            for (int k = 0; k < solution.count_routes(); ++k) {
                if (solution.load(k) + problem.demand(*iter) <= problem.capacity()) {
                    for (int idx = 1; idx <= solution.count_nodes(k) + 1; ++idx) {
                        int node = *iter;
                        double cost = problem.cost(solution.get_node(k, idx - 1), node)
                                + problem.cost(node, solution.get_node(k, idx))
                                - problem.cost(solution.get_node(k, idx - 1), solution.get_node(k, idx))
                                - gamma * (problem.cost(0, node) + problem.cost(node, 0));

                        if (cost < best_cost) {
                            best_iter = iter;
                            best_cost = cost;
                            best_k = k;
                            best_idx = idx;
                        }
                    }
                }
            }
        }

        int node = *best_iter;
        candidate_list.erase(best_iter);
        solution.insert_node(best_k, best_idx, node);

        if (best_k == solution.count_routes() - 1) {
            solution.add_route();
        }
    }

    return solution;
}

#endif

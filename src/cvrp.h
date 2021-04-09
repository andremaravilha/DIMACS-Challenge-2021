#ifndef DIMACS2021_PROBLEM_H
#define DIMACS2021_PROBLEM_H

#include <cstddef>
#include <algorithm>
#include <string>
#include <cmath>
#include <sstream>
#include <ostream>
#include <iostream>
#include <type_traits>
#include <cassert>
#include <tuple>
#include <iomanip>
#include <stdexcept>

#include "utils.h"


namespace orcs {

    template <typename cost_t> class Problem;
    template <typename cost_t> class Solution;


    /**
     * The Capacitated Vehicle Routing Problem (CVRP).
     */
    template <typename cost_t>
    class Problem {

    public:

        /**
         * Type used to represent the cost.
         */
        using cost_type = cost_t;

        /**
         * Constructor.
         * @param filename Path to the file containing the problem data.
         */
        Problem(const std::string& filename);

        /**
         * Destructor.
         */
        ~Problem();

        /**
         * Return the dimension of this instance, ie the number of customer nodes plus one (the depot).
         * @return The number of nodes (including the depot).
         */
        int dimension() const;

        /**
         * Return the the capacity maximum of the vehicles.
         * @return The maximum capacity of the vehicles.
         */
        double capacity() const;

        /**
         * Return the demand of customer i.
         * @return The demand of customer i.
         */
        double demand(int i) const;

        /**
         * Returns the cost from traveling from node i to node j.
         * @return The cost from traveling from node i to node j.
         */
        cost_t cost(int i, int j) const;

        /**
         * Return a maximum number of nodes (without considering the depot) that can be added to a route without
         * violating the capacity constraint. Note that routes may violate the capacity constraint even with a smaller
         * number of nodes. However, instances with a number of nodes greater than this limit will always violate the
         * capacity constraint.
         * @return The maximum number of nodes a route may attend without violating capacity constraint.
         */
        int max_nodes_per_route() const;

        /**
         * Check if a solution is feasible.
         * @param solution A solution
         * @param msg A (optional) pointer to a string object. Is solution is infeasible, a description of infeasibility
         * is assigned to this string.
         * @return True if solution is feasible, false otherwise.
         */
        bool is_feasible(const Solution<cost_t>& solution, std::string* msg = nullptr) const;

        /**
         * Returns an new empty solution linked to this instance of the problem.
         * @return An empty solution.
         */
        Solution<cost_t> create_empty_solution() const;

    private:

        // Instance data
        int dimension_;
        double capacity_;
        double* demand_;
        double* x_;
        double* y_;

        // Auxiliary data
        int max_nodes_per_route_;

    };


    /**
     * A solution (set of routes) for the CVRP.
     */
    template <typename cost_t>
    class Solution {

    public:

        /**
         * Constructor.
         * @param problem Instance of the CVRP.
         */
        Solution(const Problem<cost_t>& problem);

        /**
         * Copy constructor.
         * @param o Object to be copied.
         */
        Solution(const Solution<cost_t>& o);

        /**
         * Move constructor.
         * @param o Object to move.
         */
        Solution(Solution<cost_t>&& o);

        /**
         * Destructor.
         */
        ~Solution();

        /**
         * Assignment operator.
         * @param o Object to be copied.
         * @return A reference to this.
         */
        Solution& operator = (const Solution<cost_t>& o);

        /**
         * Move operator.
         * @param o Object to move.
         * @return A reference to this.
         */
        Solution& operator = (Solution<cost_t>&& o);

        /**
         * Return the number of opened routes in this solution.
         * @return The number of routes
         */
        int count_routes() const;

        /**
         * Return the number of nodes served on a specific route.
         * @param k Route index.
         * @return The number of nodes served by route k.
         */
        int count_nodes(int k) const;

        /**
         * Total cost of this solution.
         * @return Total cost of this solution.
         */
        cost_t cost() const;

        /**
         * Return the cost of a specific route.
         * @param k Route index.
         * @return Cost of route k.
         */
        cost_t cost(int k) const;

        /**
         * Load carried by the vehicle of specific route.
         * @param k Route index.
         * @return Load carried by the vehicle of route k.
         */
        double load(int k) const;

        /**
         * Create an new (empty) route.
         * @return The index of the new route.
         */
        int add_route();

        /**
         * Remove a specific route.
         * @param k Index of the route to remove.
         */
        void remove_route(int k);

        /**
         * Return the node at a specific position of a route.
         * @param k Route index.
         * @param index Index of a position in the route.
         * @return The node.
         */
        int get_node(int k, int index) const;

        /**
         * Change the node assigned to a specific position of a route.
         * @param k Route index.
         * @param index Index of a position on the route.
         * @param node Node to assign to the position on the route.
         */
        void set_node(int k, int index, int node);

        /**
         * Insert a node at the end of the route.
         * @param k Route index.
         * @param node Node.
         */
        void insert_node(int k, int node);

        /**
         * Insert a node at a specific position of a route.
         * @param k Route index.
         * @param index Index of a position on the route.
         * @param node Node.
         */
        void insert_node(int k, int index, int node);

        /**
         * Remove the node at a specific position on the route.
         * @param k Route index.
         * @param index Index of a position on the route.
         */
        void remove_node(int k, int index);

        /**
         * Reverse the order of the nodes in the range [start, end].
         * @param k Route index.
         * @param start Initial position.
         * @param end Final position.
         */
        void reverse_route(int k, int start, int end);

        /**
         * Write this solution into a file following the CVRPLIB format.
         * @param filename Path to file in which the solution should be written.
         */
        void export_CVRPLIB(const std::string& filename) const;

        /**
         * Write this solution into an output stream.
         * @param os Output stream.
         */
        void export_CVRPLIB(std::ostream& os) const;

    private:

        const Problem<cost_t>& problem_;
        int** routes_;
        double* load_;
        int* n_nodes_;
        int n_routes_;
        cost_t* cost_;
        cost_t total_cost_;

    };

}


/**
 * Print a solution to an output stream.
 * @param os Output stream.
 * @param solution A solution
 * @return A reference to the output stream.
 */
template <typename cost_t>
std::ostream& operator << (std::ostream& os, const orcs::Solution<cost_t>& solution);


/*
 * Implementation: CVRP class
 */

template <typename cost_t>
orcs::Problem<cost_t>::Problem(const std::string& filename) :
dimension_(0), capacity_(0), demand_(nullptr), x_(nullptr), y_(nullptr), max_nodes_per_route_(0)
{
    // Open instance file
    std::ifstream ifs(filename);
    if (ifs.is_open()) {

        std::string line;
        std::vector<std::string> tokens;
        std::string token;

        while(std::getline(ifs, line)) {

            // Check if "Node coordinators" section has been reached
            if (line.find("NODE_COORD_SECTION") != std::string::npos) {
                break;
            }

            // Split line
            orcs::utils::split(line, ':', tokens);

            // Dimension
            if (tokens[0].find("DIMENSION") != std::string::npos) {
                dimension_ = std::stoi(tokens[1]);
            }

            // Capacity
            if (tokens[0].find("CAPACITY") != std::string::npos) {
                capacity_ = std::stoi(tokens[1]);
            }
        }

        // Node coordinates
        x_ = new double[dimension_];
        y_ = new double[dimension_];

        for (int i = 0; i < dimension_; ++i) {
            std::getline(ifs, line);
            std::istringstream iss(line);

            // skip node id
            iss >> token;

            // x-coordinate
            iss >> token;
            x_[i] = std::stod(token);

            // y-coordinate
            iss >> token;
            y_[i] = std::stod(token);
        }

        // Demands
        std::getline(ifs, line);  // skip title of demand section
        demand_ = new double[dimension_];
        double* sorted_demands = new double[dimension_];
        for (int i = 0; i < dimension_; ++i) {

            // get line
            std::getline(ifs, line);
            std::istringstream iss(line);

            // skip node id
            iss >> token;

            // demand
            iss >> token;
            demand_[i] = std::stod(token);
            sorted_demands[i] = demand_[i];
        }

        // Sort demand to calculate the maximum number of nodes in a route
        std::sort(sorted_demands, sorted_demands + dimension_);
        max_nodes_per_route_ = 0;
        double load = 0.0;
        for (int i = 0; i < dimension_; ++i) {
            load += sorted_demands[i];
            if (load <= capacity_) {
                max_nodes_per_route_++;
            }
        }

        delete[] sorted_demands;

        // Close instance file
        ifs.close();
    } else {
        std::string msg = "Couldn't read instance file \"" + filename + "\"";
        throw std::runtime_error(msg);
    }
}


template <typename cost_t>
orcs::Problem<cost_t>::~Problem() {
    if (demand_ != nullptr) delete[] demand_;
    if (x_ != nullptr) delete[] x_;
    if (y_ != nullptr) delete[] y_;
}


template <typename cost_t>
inline
int orcs::Problem<cost_t>::dimension() const {
    return dimension_;
}


template <typename cost_t>
inline
double orcs::Problem<cost_t>::capacity() const {
    return capacity_;
}


template <typename cost_t>
inline
double orcs::Problem<cost_t>::demand(int i) const {
    assert(("Invalid node index", i >= 0 && i < dimension_));
    return demand_[i];
}


template <>
inline
int orcs::Problem<int>::cost(int i, int j) const {
    assert(("Invalid node index", i >= 0 && i < dimension_ && j >= 0 && j < dimension_));
    return (int) (std::sqrt(((x_[i] - x_[j]) * (x_[i] - x_[j])) + ((y_[i] - y_[j]) * (y_[i] - y_[j]))) + 0.5);
}


template <>
inline
long orcs::Problem<long>::cost(int i, int j) const {
    assert(("Invalid node index", i >= 0 && i < dimension_ && j >= 0 && j < dimension_));
    return (long) (std::sqrt(((x_[i] - x_[j]) * (x_[i] - x_[j])) + ((y_[i] - y_[j]) * (y_[i] - y_[j]))) + 0.5);
}


template <typename cost_t>
inline
cost_t orcs::Problem<cost_t>::cost(int i, int j) const {
    assert(("Invalid node index", i >= 0 && i < dimension_ && j >= 0 && j < dimension_));
    return std::sqrt(((x_[i] - x_[j]) * (x_[i] - x_[j])) + ((y_[i] - y_[j]) * (y_[i] - y_[j])));
}


template <typename cost_t>
inline
int orcs::Problem<cost_t>::max_nodes_per_route() const {
    return max_nodes_per_route_;
}


template <typename cost_t>
bool orcs::Problem<cost_t>::is_feasible(const Solution<cost_t>& solution, std::string* msg) const {

    std::string aux_msg = "";
    if (msg == nullptr) msg = &aux_msg;

    // Checks if routes start and end at depot node, and check for invalid customer nodes
    for (int k = 0; k < solution.count_routes(); ++k) {

        // Route starts at depot
        if (solution.get_node(k, 0) != 0) {
            *msg = orcs::utils::format("Route #%d does not start at depot node.", k);
            return false;
        }

        // Route ends at depot
        if (solution.get_node(k, solution.count_nodes(k) + 1) != 0) {
            *msg = orcs::utils::format("Route #%d does not end at depot node.", k);
            return false;
        }

        // Counts the number of occurrences of each customer
        for (int idx = 1; idx <= solution.count_nodes(k); ++idx) {
            int node = solution.get_node(k, idx);
            if (node < 1 || node >= dimension_) {
                *msg = orcs::utils::format("Node %d (on route %d) is not a valid customer node (expecting values from 1 to %d).", node, k, dimension_ - 1);
                return false;
            }
        }
    }

    // Check if customer nodes are visited exactly once
    int* count = new int[dimension_];
    for (int i = 0; i < dimension_; ++i) {
        count[i] = 0;
    }

    for (int k = 0; k < solution.count_routes(); ++k) {
        for (int idx = 1; idx <= solution.count_nodes(k); ++idx) {
            count[solution.get_node(k, idx)] += 1;
        }
    }

    for (int i = 1; i < dimension_; ++i) {
        if (count[i] != 1) {
            *msg = orcs::utils::format("Customer %d is visited %d times.", i, count[i]);
            return false;
        }
    }

    delete[] count;

    // Check load on vehicles and costs
    cost_t total_cost = 0;
    for (int k = 0; k < solution.count_routes(); ++k) {
        double load_k = 0.0;
        cost_t cost_k = 0;

        // Calculate load
        for (int idx = 1; idx <= solution.count_nodes(k); ++idx) {
            load_k += demand_[solution.get_node(k, idx)];
        }

        // Calculate cost
        for (int idx = 0; idx < solution.count_nodes(k) + 1; ++idx) {
            cost_k += cost(solution.get_node(k, idx), solution.get_node(k, idx + 1));
        }
        total_cost += cost_k;

        assert(("Calculated value of cost does not correspond with the expected.", std::abs(cost_k - solution.cost(k)) < 1e-5));
        assert(("Calculated value of load does not correspond with the expected.", std::abs(load_k - solution.load(k)) < 1e-5));

        if (load_k > capacity_) {
            *msg = orcs::utils::format("Load on route #%d is greater than vehicle's capacity (load: %.4lf, capacity: %.4lf).", k, load_k, capacity_);
            return false;
        }
    }

    assert(("Calculated value of total cost does not correspond with the expected.", std::abs(total_cost - solution.cost()) < 1e-5));

    // All constraints are satisfied
    *msg = "All constraints are satisfied.";
    return true;
}


template <typename cost_t>
inline
orcs::Solution<cost_t> orcs::Problem<cost_t>::create_empty_solution() const {
    Solution<cost_t> solution(*this);
    return solution;
}


/*
 * Implementation: Solution class
 */

template <typename cost_t>
orcs::Solution<cost_t>::Solution(const Problem<cost_t>& problem) :
problem_(problem), n_routes_(0), total_cost_(0), routes_(new int*[problem_.dimension() - 1]),
n_nodes_(new int[problem_.dimension() - 1]), load_(new double[problem_.dimension() - 1]),
cost_(new cost_t[problem_.dimension() - 1])
{
    // Do nothing here.
}


template <typename cost_t>
orcs::Solution<cost_t>::Solution(const Solution<cost_t>& o) : Solution(o.problem_)
{
    total_cost_ = o.total_cost_;
    n_routes_ = o.n_routes_;
    for (int k = 0; k < o.n_routes_; ++k) {
        n_nodes_[k] = o.n_nodes_[k];
        load_[k] = o.load_[k];
        cost_[k] = o.cost_[k];

        routes_[k] = new int[problem_.max_nodes_per_route() + 2];
        for (int i = 0; i < o.n_nodes_[k] + 2; ++i) {
            routes_[k][i] = o.routes_[k][i];
        }
    }
}


template <typename cost_t>
orcs::Solution<cost_t>::Solution(Solution<cost_t>&& o) :
problem_(o.problem_), n_routes_(0), total_cost_(0), routes_(nullptr), n_nodes_(nullptr), load_(nullptr), cost_(nullptr)
{
    // Transfer data to this object
    total_cost_ = o.total_cost_;
    n_routes_ = o.n_routes_;
    routes_ = o.routes_;
    n_nodes_ = o.n_nodes_;
    load_ = o.load_;
    cost_ = o.cost_;

    // Remove references to the data from the moved object
    o.n_routes_ = 0;
    o.total_cost_ = 0;
    o.routes_ = nullptr;
    o.n_nodes_ = nullptr;
    o.load_ = nullptr;
    o.cost_ = nullptr;
}


template <typename cost_t>
orcs::Solution<cost_t>::~Solution() {
    if (routes_ != nullptr) {
        for (int k = 0; k < n_routes_; ++k) {
            delete[] routes_[k];
        }
        delete[] routes_;
    }

    if (n_nodes_ != nullptr) delete[] n_nodes_;
    if (load_ != nullptr) delete[] load_;
    if (cost_ != nullptr) delete[] cost_;
}


template <typename cost_t>
orcs::Solution<cost_t>& orcs::Solution<cost_t>::operator = (const Solution<cost_t>& o) {
    assert(("Assignment between solutions of different instances of the problem.", &problem_ == &(o.problem_)));

    // Avoid self-assignment
    if (this != &o) {

        // Free resources of this object
        if (routes_ != nullptr) {
            for (int k = 0; k < n_routes_; ++k) {
                delete[] routes_[k];
            }
        }

        // Copy data
        total_cost_ = o.total_cost_;
        n_routes_ = o.n_routes_;
        for (int k = 0; k < o.n_routes_; ++k) {
            n_nodes_[k] = o.n_nodes_[k];
            load_[k] = o.load_[k];
            cost_[k] = o.cost_[k];

            routes_[k] = new int[problem_.max_nodes_per_route() + 2];
            for (int i = 0; i < o.n_nodes_[k] + 2; ++i) {
                routes_[k][i] = o.routes_[k][i];
            }
        }
    }

    return *this;
}


template <typename cost_t>
orcs::Solution<cost_t>& orcs::Solution<cost_t>::operator = (Solution<cost_t>&& o) {
    assert(("Assignment between solutions of different instances of the problem.", &problem_ == &(o.problem_)));

    // Avoid self-assignment
    if (this == &o) {

        // Free resources of this object
        if (routes_ != nullptr) {
            for (int k = 0; k < n_routes_; ++k) {
                delete[] routes_[k];
            }
            delete[] routes_;
        }

        if (n_nodes_ != nullptr) delete[] n_nodes_;
        if (load_ != nullptr) delete[] load_;
        if (cost_ != nullptr) delete[] cost_;

        // Transfer data to this object
        total_cost_ = o.total_cost_;
        n_routes_ = o.n_routes_;
        routes_ = o.routes_;
        n_nodes_ = o.n_nodes_;
        load_ = o.load_;
        cost_ = o.cost_;

        // Remove references to the data from the moved object
        o.n_routes_ = 0;
        o.total_cost_ = 0;
        o.routes_ = nullptr;
        o.n_nodes_ = nullptr;
        o.load_ = nullptr;
        o.cost_ = nullptr;
    }

    return *this;
}


template <typename cost_t>
inline
int orcs::Solution<cost_t>::count_routes() const {
    return n_routes_;
}


template <typename cost_t>
inline
int orcs::Solution<cost_t>::count_nodes(int k) const {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    return n_nodes_[k];
}


template <typename cost_t>
inline
cost_t orcs::Solution<cost_t>::cost() const {
    return total_cost_;
}


template <typename cost_t>
inline
cost_t orcs::Solution<cost_t>::cost(int k) const {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    return cost_[k];
}


template <typename cost_t>
inline
double orcs::Solution<cost_t>::load(int k) const {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    return load_[k];
}


template <typename cost_t>
int orcs::Solution<cost_t>::add_route() {
    assert(("Maximum number of routes is violated.", n_routes_ < problem_.dimension() - 1));
    routes_[n_routes_] = new int[problem_.max_nodes_per_route() + 2];
    routes_[n_routes_][0] = 0;
    routes_[n_routes_][1] = 0;
    n_nodes_[n_routes_] = 0;
    load_[n_routes_] = 0;
    cost_[n_routes_] = 0;
    ++n_routes_;
    return (n_routes_ - 1);
}


template<typename cost_t>
void orcs::Solution<cost_t>::remove_route(int k) {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    total_cost_ -= cost_[k];
    delete[] routes_[k];
    for (int idx = k; idx < n_routes_ - 1; ++idx) {
        routes_[idx] = routes_[idx + 1];
        n_nodes_[idx] = n_nodes_[idx + 1];
        load_[idx] = load_[idx + 1];
        cost_[idx] = cost_[idx + 1];
    }
    --n_routes_;
}


template <typename cost_t>
inline
int orcs::Solution<cost_t>::get_node(int k, int index) const {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    assert(("Invalid node index", index >= 0 && index < n_nodes_[k] + 2));
    return routes_[k][index];
}


template <typename cost_t>
void orcs::Solution<cost_t>::set_node(int k, int index, int node) {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    assert(("Invalid node index", index >= 1 && index <= n_nodes_[k]));

    if (node < 1 || node >= problem_.dimension()) {
        std::cout << "NODE: " << node << std::endl;
    }

    assert(("Invalid node", node >= 1 && node < problem_.dimension()));

    double delta_load = problem_.demand(node) - problem_.demand(routes_[k][index]);

    cost_t delta_cost = problem_.cost(routes_[k][index - 1], node) + problem_.cost(node, routes_[k][index + 1])
            - problem_.cost(routes_[k][index - 1], routes_[k][index])
            - problem_.cost(routes_[k][index], routes_[k][index + 1]);

    routes_[k][index] = node;
    load_[k] += delta_load;
    cost_[k] += delta_cost;
    total_cost_ += delta_cost;
}


template <typename cost_t>
inline
void orcs::Solution<cost_t>::insert_node(int k, int node) {
    insert_node(k, n_nodes_[k] + 1, node);
}


template <typename cost_t>
void orcs::Solution<cost_t>::insert_node(int k, int index, int node) {
    assert(("Index extrapolates the length of the array", index >= 0 && index < problem_.max_nodes_per_route() + 2));
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    assert(("Invalid node index", index >= 1 && index <= n_nodes_[k] + 1));
    assert(("Invalid node", node >= 1 && node < problem_.dimension()));

    double delta_load = problem_.demand(node);

    cost_t delta_cost = problem_.cost(routes_[k][index - 1], node) + problem_.cost(node, routes_[k][index])
                        - problem_.cost(routes_[k][index - 1], routes_[k][index]);

    for (int i = n_nodes_[k] + 1; i >= index; --i) {
        routes_[k][i + 1] = routes_[k][i];
    }

    routes_[k][index] = node;
    load_[k] += delta_load;
    cost_[k] += delta_cost;
    total_cost_ += delta_cost;
    n_nodes_[k] += 1;
}


template <typename cost_t>
void orcs::Solution<cost_t>::remove_node(int k, int index) {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    assert(("Invalid node index", index >= 1 && index <= n_nodes_[k]));

    int node = routes_[k][index];

    double delta_load = -problem_.demand(node);

    cost_t delta_cost = problem_.cost(routes_[k][index - 1], routes_[k][index + 1])
            - problem_.cost(routes_[k][index - 1], node) - problem_.cost(node, routes_[k][index + 1]);

    for (int i = index; i <= n_nodes_[k]; ++i) {
        routes_[k][i] = routes_[k][i + 1];
    }

    load_[k] += delta_load;
    cost_[k] += delta_cost;
    total_cost_ += delta_cost;
    n_nodes_[k] -= 1;
}


template <typename cost_t>
void orcs::Solution<cost_t>::reverse_route(int k, int start, int end) {
    assert(("Invalid route index.", k >= 0 && k < n_routes_));
    assert(("Invalid node index", start >= 1 && start <= n_nodes_[k]));
    assert(("Invalid node index", end >= 1 && end <= n_nodes_[k]));
    assert(("Start index cannot be greater than end index", start <= end));
    cost_t delta_cost = problem_.cost(routes_[k][start - 1], routes_[k][end])
            + problem_.cost(routes_[k][start], routes_[k][end + 1])
            - problem_.cost(routes_[k][start - 1], routes_[k][start])
            - problem_.cost(routes_[k][end], routes_[k][end + 1]);
    cost_[k] += delta_cost;
    total_cost_ += delta_cost;
    std::reverse(routes_[k] + start, routes_[k] + end + 1);
}


template <typename cost_t>
void orcs::Solution<cost_t>::export_CVRPLIB(const std::string& filename) const {
    std::ofstream file(filename, std::ios_base::out | std::ios_base::trunc);
    export_CVRPLIB(file);
    file.close();
}


template <typename cost_t>
void orcs::Solution<cost_t>::export_CVRPLIB(std::ostream& os) const {
    for (int k = 0; k < n_routes_; ++k) {
        os << "Route #" << (k + 1) << ":";
        for (int i = 1; i <= n_nodes_[k]; ++i) {
            os << " " << routes_[k][i];
        }
        os << std::endl;
    }
    os << "Cost " << std::fixed << std::setprecision(6) << total_cost_ << std::endl;
}


/*
 * Implementation: Other classes and functions
 */

template <typename cost_t>
std::ostream& operator << (std::ostream& os, const orcs::Solution<cost_t>& solution) {
    solution.export_CVRPLIB(os);
    return os;
}


#endif

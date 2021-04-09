#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <stdexcept>
#include <cxxtimer.hpp>
#include <cxxopts.hpp>

#include "cvrp.h"
#include "heuristics.h"
#include "moves.h"


/**
 * Team identification in the competition.
 */
const char* TEAM_ID = "ORCS";


/*
 * Function declarations.
 */

template <typename cost_t>
void solve(const cxxopts::ParseResult& options, const cxxtimer::Timer& timer);


/*
 * Implementation.
 */

template <typename cost_t>
void solve(const cxxopts::ParseResult& options, const cxxtimer::Timer& timer) {

    // Initialize the random number generator
    std::mt19937 generator(options["seed"].as<int>());

    // Get vebosity and time limit settings
    bool verbose = (options.count("non-verbose") > 0 ? false : true);
    double time_limit = options["time-limit"].as<double>();

    // Load problem data
    orcs::Problem<cost_t> problem(options["instance"].as<std::string>());

    // Create a starting solution through a constructive heuristic
    auto start = orcs::modified_cheapest_insertion_constructive(problem, generator);

    // Moves to be used at local search-based heuristic
    orcs::Shift_1 shift_1(problem, generator);
    orcs::Swap_1_1 swap_1_1(problem, generator);
    orcs::Exchange exchange(problem, generator);
    orcs::OrOpt1 or_opt1(problem, generator);
    orcs::TwoOpt twoOpt(problem, generator);
    orcs::ThreeOpt threeOpt(problem, generator);

    std::vector<orcs::Move*> moves = { &shift_1, &swap_1_1, &exchange, &or_opt1, &twoOpt, &threeOpt };
    //std::vector<orcs::Move*> moves = { &shift_1, &exchange, &or_opt1, &twoOpt };

    // Parameters for the simulated annealing heuristic
    double alpha = 0.97;
    double t0 = 1.0;
    unsigned long long sa_max = 1176628;

    // Improve the staring solution
    auto solution = orcs::simulated_annealing(problem, start, moves, generator, timer, time_limit, verbose, alpha, t0, sa_max);
    //auto solution = orcs::iterated_local_search(problem, start, moves, generator, timer, time_limit, verbose);

    // Show output data if verbose mode is disabled
    if (!verbose) {

        // Get runtime (in seconds)
        double runtime = timer.count<std::chrono::milliseconds>() / 1000.0;

        // Check solution's feasibility
        std::string feasibility = "";
        problem.is_feasible(solution, &feasibility);

        // Show data
        std::cout << "Cost: " << std::fixed << std::setprecision(6) << solution.cost() << std::endl;
        std::cout << "Runtime (s): " << std::fixed << std::setprecision(6) << runtime << std::endl;
        std::cout << "Feasibility: " << feasibility << std::endl << std::endl;
    }

    // Export solution, if set
    if (options.count("output") > 0) {
        solution.export_CVRPLIB(options["output"].as<std::string>());
    }
}


int main(int argc, char** argv) {

    try {

        // Start timer
        cxxtimer::Timer timer(true);

        // Command-line parser
        cxxopts::Options options(argv[0], "12th DIMACS Implementation Challenge: CVRP track");

        options
            .positional_help("INSTANCE ROUNDING")
            .add_options()
                ("help", "Show this help message and exit.", cxxopts::value<bool>())
                ("non-verbose", "Disable verbose mode", cxxopts::value<bool>())
                ("time-limit", "Time limit in seconds", cxxopts::value<double>()->default_value("1E100"))
                ("seed", "Seed to initialize the random number generator", cxxopts::value<int>()->default_value("0"))
                ("output", "Path to write the best solution found.", cxxopts::value<std::string>())
                ("instance", "Path to instance file.", cxxopts::value<std::string>())
                ("rounding", "Use 1 to round costs, 0 otherwise.", cxxopts::value<int>())
                ("positional", "Other positional arguments", cxxopts::value<std::vector<std::string>>());

        options.parse_positional({"instance", "rounding", "positional"});
        auto args = options.parse(argc, argv);

        if (args.count("help")) {
            std::cout << options.help() << std::endl;
            return EXIT_SUCCESS;
        }

        // Check if INSTANCE and ROUNDING arguments were set
        if (args.count("instance") == 0 || args.count("rounding") == 0) {
            throw std::invalid_argument("Missing arguments.");
        }

        // Run the solver
        if (args["rounding"].as<int>() == 1) {
            solve<int>(args, timer);
        } else {
            solve<double>(args, timer);
        }

    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << "Type the following command for a correct usage." << std::endl;
        std::cerr << argv[0] << " --help" << std::endl << std::endl;
        return EXIT_FAILURE;

    } catch (...) {
        std::cerr << "Unexpected error." << std::endl;
        std::cerr << "Type the following command for a correct usage." << std::endl;
        std::cerr << argv[0] << " --help" << std::endl << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

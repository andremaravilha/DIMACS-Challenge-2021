#ifndef DIMACS2021_UTILS_H
#define DIMACS2021_UTILS_H

#include <cstddef>
#include <cmath>
#include <random>
#include <string>
#include <tuple>
#include <iostream>


namespace orcs {

    namespace utils {

        /**
         * The threshold used when compared equality of floating point values.
         */
        constexpr double THRESHOLD = 1e-6;

        /**
         * Compare two numbers (of different types). When at least one of the arguments is a floating point value, the
         * values are considered equal if the absolute value of their difference is small than a threshold value. See
         * the constant THRESHOLD.
         *
         * @param first A number.
         * @param second A number.
         *
         * @return Return -1 if first is less then second, 0 if both numbers are equal, 1 if less is greater than
         * second.
         */
        template <typename T1, typename T2>
        int compare(T1 first, T2 second);

        /**
         * Compare two numbers (of same type). When the values are floating point values, the values are considered
         * equal if the absolute value of their difference is small than a threshold value. See the constant THRESHOLD.
         *
         * @param first A number.
         * @param second A number.
         *
         * @return Return -1 if first is less then second, 0 if both numbers are equal, 1 if less is greater than
         * second.
         */
        template <typename T1>
        int compare(T1 first, T1 second);

        /**
         * Compare equality between two numbers. When the values are floating point values, the values are considered
         * equal if the absolute value of their difference is small than a threshold value. See the constant THRESHOLD.
         *
         * @param first A number.
         * @param second A number.
         *
         * @return True if the numbers are equal, false otherwise.
         */
        template <typename T1, typename T2>
        bool is_equal(T1 first, T2 second);

        /**
         * Check if the first number is greater than the second one. When the values are floating point values, the
         * values are considered equal if the absolute value of their difference is small than a threshold value. See
         * the constant THRESHOLD.
         *
         * @param first A number.
         * @param second A number.
         *
         * @return True if first is greater than second, false otherwise.
         */
        template <typename T1, typename T2>
        bool is_greater(T1 first, T2 second);

        /**
         * Check if the first number is lower than the second one. When the values are floating point values, the values
         * are considered equal if the absolute value of their difference is small than a threshold value. See the
         * constant THRESHOLD.
         *
         * @param first A number.
         * @param second A number.
         *
         * @return True if first is lower than second, false otherwise.
         */
        template <typename T1, typename T2>
        bool is_lower(T1 first, T2 second);

        /**
         * Check if the first number is greater or equal to the second one. When the values are floating point values,
         * the values are considered equal if the absolute value of their difference is small than a threshold value.
         * See the constant THRESHOLD.
         *
         * @param first A number.
         * @param second A number.
         *
         * @return True if first is greater or equal to the second number, false otherwise.
         */
        template <typename T1, typename T2>
        bool is_greater_equal(T1 first, T2 second);

        /**
         * Check if the first number is lower of equal to the second one.When the values are floating point values, the
         * values are considered equal if the absolute value of their difference is small than a threshold value. See
         * the constant THRESHOLD.
         *
         * @param first A number.
         * @param second A number.
         *
         * @return True if first is lower or equal to the second number, false otherwise.
         */
        template <typename T1, typename T2>
        bool is_lower_equal(T1 first, T2 second);

        /**
         * String formatter. If str includes format specifiers (subsequences beginning with %), the additional arguments
         * following str are formatted and inserted in the resulting string replacing their respective specifiers.
         *
         * @param str A string to format.
         * @param args Arguments used to format the string.
         *
         * @return The formatted string
         */
        template <class... T>
        std::string format(const std::string& str, T... args);

        /**
         * Return a container of all substrings in the string, using a separator as the delimiter string.
         * @param str A string to split.
         * @param separator A delimiter character
         * @param container Container to store the substrings.
         */
        template <typename container_t>
        void split(const std::string& str, char separator, container_t&& container);

        /**
         * This function randomly chooses a element from the container values accordingly to their
         * respective weights.
         *
         * @tparam  TObject
         * @tparam  TContainer1
         * @tparam  TContainer2
         * @tparam  TRandom
         * @param   values
         * @param   weights
         * @param   generator
         * @return
         */
        template <class TObject, template<class, class...> class TContainer1, template<class, class...> class TContainer2, class TRandom>
        std::tuple<TObject, std::size_t> choose(const TContainer1<TObject>& values, const TContainer2<double>& weights, TRandom& generator);

    }

}


/*
 * Function definition.
 */

template <typename T1, typename T2>
inline
int orcs::utils::compare(T1 first, T2 second) {
    return std::abs(first - second) < THRESHOLD ? 0 : (first < second ? -1 : 1);
}


template <typename T1>
inline
int orcs::utils::compare(T1 first, T1 second) {
    return std::abs(first - second) < THRESHOLD ? 0 : (first < second ? -1 : 1);
}


template <>
inline
int orcs::utils::compare(int first, int second) {
    return first == second ? 0 : (first < second ? -1 : 1);
}


template <>
inline
int orcs::utils::compare(long first, long second) {
    return first == second ? 0 : (first < second ? -1 : 1);
}


template <typename T1, typename T2>
inline
bool orcs::utils::is_equal(T1 first, T2 second) {
    return compare(first, second) == 0;
}


template <typename T1, typename T2>
inline
bool orcs::utils::is_greater(T1 first, T2 second) {
    return compare(first, second) == 1;
}


template <typename T1, typename T2>
inline
bool orcs::utils::is_lower(T1 first, T2 second) {
    return compare(first, second) == -1;
}


template <typename T1, typename T2>
inline
bool orcs::utils::is_greater_equal(T1 first, T2 second) {
    return compare(first, second) != -1;
}


template <typename T1, typename T2>
inline
bool orcs::utils::is_lower_equal(T1 first, T2 second) {
    return compare(first, second) != 1;
}


template <class... T>
std::string orcs::utils::format(const std::string& str, T... args) {

    // Define the buffer size
    std::size_t buffer_size = std::max(static_cast<std::size_t>(1024),
                                       static_cast<std::size_t>(2 * str.length()));

    // Create the formatted string
    char* buffer = new char[buffer_size];
    sprintf(buffer, str.c_str(), args...);

    // Return the formatted string as a std::string object
    return std::string(buffer);
}


template <typename container_t>
void orcs::utils::split(const std::string& str, char separator, container_t&& container) {
    container.clear();
    std::size_t start = 0;
    std::size_t current = 0;
    for (current = 0; current < str.length(); ++current) {
        if (str.at(current) == separator) {
            container.push_back(str.substr(start, current - start));
            start = current + 1;
        }
    }
    container.push_back(str.substr(start, current - start));
}


template <class TObject, template<class, class...> class TContainer1, template<class, class...> class TContainer2, class TRandom>
std::tuple<TObject, std::size_t> orcs::utils::choose(const TContainer1<TObject>& values, const TContainer2<double>& weights, TRandom& generator) {

    // Sum all weights
    double sum = 0.0;
    for (const auto& w : weights) {
        sum += w;
    }

    // Get a random number
    double random_value = (generator() / (double) generator.max()) * sum;

    // Choose a value
    std::size_t index = 0;
    auto iter_values = values.begin();
    auto iter_weights = weights.begin();

    double aux = 0.0;
    while (iter_values != values.end()) {
        if (aux + *iter_weights >= random_value) {
            break;
        } else {
            aux += *iter_weights;
            ++index;
            ++iter_values;
            ++iter_weights;
        }
    }

    // Return the chosen value
    return std::make_tuple(*iter_values, index);
}

#endif

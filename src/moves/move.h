#ifndef DIMACS2021_MOVE_H
#define DIMACS2021_MOVE_H

#include <any>
#include <string>


namespace orcs {

    /**
     * Interface implemented by moves.
     */
    class Move {
    public:

        /**
         * Return the name of this movement.
         * @return The name of this movement.
         */
        virtual const char* const name() const = 0;

        /**
         * Perform the move and returns the difference in the cost of the solution after the move. Note that this
         * instance of the move will be in an intermediate state until the movement is confirmed (calling to accept()
         * method) or reject (calling to reject() method).
         * @param solution_ptr A pointer to a solution.
         * @param intensive_search Set it to true if a intensive search must be performed, or set it to false to
         * perform a random move.
         * @return The difference in the cost of the solution after the move.
         */
        virtual std::any try_move(void* solution_ptr, bool intensive_search) = 0;

        /**
         * Perform the best move and returns the difference in the cost of the solution after the move. Note that this
         * instance of the move will be in an intermediate state until the movement is confirmed (calling to accept()
         * method) or reject (calling to reject() method).
         * @param solution_ptr A pointer to a solution.
         * @return The difference in the cost of the solution after the move.
         */
        virtual std::any try_best_move(void* solution_ptr) = 0;

        /**
         * Check if a solution satisfy the minimum requirements to try this move.
         * @param solution_ptr A solution.
         * @param intensive_search Set it to true if a intensive search must be performed, or set it to false to
         * perform a random move.
         * @return True if the solution satisfy the minimum requirements to try this move, false otherwise.
         */
        virtual bool has_move(void* solution_ptr, bool intensive_search) = 0;

        /**
        * Accept the move.
        */
        virtual void accept() = 0;

        /**
         * Reject the move.
         */
        virtual void reject() = 0;

    };

}

#endif

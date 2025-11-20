#pragma once

#include <cstddef>
namespace FOC {

template <typename T, size_t N>
class MovingAverage {
   public:
    /**
     * @brief Default constructor. Initializes the filter to a zero state.
     */
    MovingAverage() : sum(0), count(0), index(0) {
        for (size_t i = 0; i < N; ++i) {
            window[i] = 0;
        }
    }

    /**
     * @brief Adds a new value to the moving average filter.
     *
     * @param newValue The new value to add.
     */
    void add(T newValue) {
        if (count < N) {
            // Buffer is not yet full
            count++;
        } else {
            // Subtract the oldest value from the sum
            sum -= window[index];
        }

        // Add the new value to the window and the sum
        window[index] = newValue;
        sum += newValue;

        // Move to the next index in the circular buffer
        index = (index + 1) % N;
    }

    /**
     * @brief Gets the current moving average.
     *
     * @return The calculated moving average. Returns 0 if no values have been
     * added.
     */
    T get() const {
        if (count == 0) {
            return 0;
        }
        return sum / static_cast<T>(count);
    }

    /**
     * @brief Resets the filter to its initial state.
     */
    void reset() {
        sum = 0;
        count = 0;
        index = 0;
        for (size_t i = 0; i < N; ++i) {
            window[i] = 0;
        }
    }

   private:
    T window[N];   // Circular buffer to store the values
    T sum;         // Sum of the current values in the window
    size_t count;  // Number of values currently in the window
    size_t index;  // Current index in the circular buffer
};

}  // namespace FOC

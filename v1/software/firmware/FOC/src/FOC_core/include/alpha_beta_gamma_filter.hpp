namespace FOC {
/**
 * @class AlphaBetaGammaFilter
 * @brief Implements a constant acceleration Alpha-Beta-Gamma filter.
 *
 * This filter is used to track and smooth position, velocity, and acceleration
 * from noisy position measurements. It assumes a constant acceleration model,
 * making it ideal for smoothing signals in systems where acceleration changes
 * are not excessively abrupt (or for providing smooth estimates at constant
 * velocity).
 */
class AlphaBetaGammaFilter {
   public:
    /**
     * @brief Constructor for the AlphaBetaGammaFilter.
     *
     * @param alpha The gain for the position correction.
     * @param beta  The gain for the velocity correction.
     * @param gamma The gain for the acceleration correction.
     * @param dt    The constant time step (sampling period) in seconds.
     * @param x0    The initial position of the system.
     */
    AlphaBetaGammaFilter(float alpha, float beta, float gamma, float dt,
                         float x0 = 0.0)
        : alpha_(alpha),
          beta_(beta),
          gamma_(gamma),
          v_correction_(beta / dt),
          a_correction_(2.0f * gamma / (dt * dt)),
          dt_(dt),
          xk_(x0),   // Initial position estimate
          vk_(0.0),  // Initial velocity estimate
          ak_(0.0)   // Initial acceleration estimate
    {}

    /**
     * @brief Performs one update step of the filter.
     *
     * @param measurement The new position measurement from the encoder.
     * @return The updated and filtered velocity estimate.
     */
    float update(float measurement) {
        // --- Prediction Step ---
        // Predict the next state based on the current state and the constant
        // acceleration model.
        float x_pred = xk_ + (vk_ * dt_) + (0.5 * ak_ * dt_ * dt_);
        float v_pred = vk_ + (ak_ * dt_);
        float a_pred = ak_;

        // --- Correction Step ---
        // Calculate the innovation (the error between the measurement and the
        // prediction).
        float innovation = measurement - x_pred;

        // Update the state estimates by correcting the prediction with the
        // innovation scaled by the gains.
        xk_ = x_pred + alpha_ * innovation;
        vk_ = v_pred + v_correction_ * innovation;
        ak_ = a_pred + a_correction_ * innovation;

        return vk_;
    }

    /** @brief Resets the filter's internal state. */
    void reset(float x0 = 0.0) {
        xk_ = x0;
        vk_ = 0.0;
        ak_ = 0.0;
    }

    // --- Getter Functions ---
    float get_position() const { return xk_; }
    float get_velocity() const { return vk_; }
    float get_acceleration() const { return ak_; }

   private:
    // Filter gains
    float alpha_;
    float beta_;
    float gamma_;
    float v_correction_;
    float a_correction_;

    // Time step
    float dt_;

    // State variables (k-th estimate)
    float xk_;  // Position
    float vk_;  // Velocity
    float ak_;  // Acceleration
};
}  // namespace FOC

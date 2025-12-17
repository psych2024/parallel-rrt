#pragma once

#include <concepts>
#include <cstdint>

namespace rrt {
template<typename StateT>
/**
 * @brief Abstract base class for representing a possibly continuous state space
 *
 * State space should be a metric space that satisfies the metric axioms
 * This is generalized beyond a vector space.
 *
 */
class StateSpace {
public:
    virtual ~StateSpace() = default;

    virtual double distance(const StateT &s1, const StateT &s2) const = 0;

    virtual StateT clone(const StateT &s) const = 0;

    /**
     * @brief Function required for The branch_extend step in RRT.
     *      For example, in the case of an euclidean vector space, this would
     *      be the state in the direction of source - target, scaled wrt step_size
     *
     * @param source the source state
     * @param target
     * @return A state that is closer to source
     */
    virtual StateT
    branch_extend(const StateT &source, const StateT &target, double step_size) const = 0;

    virtual StateT
    interpolate(const StateT &from, const StateT &to, double t) const = 0;

    /**
     * Sample a random state from the state space uniformly.
     * @return Sampled state, possibly in collision
     */
    virtual StateT uniform_sample() const = 0;

    virtual bool is_in_collision(const StateT &state) const = 0;

    /**
     * Check if it is possible to connect a straight line between to and from
     * Without obstacles
     * @param from state from
     * @param to state to
     * @param step_size checking resolution
     * @return true if no collisions, false otherwise
     *
     * @note does not check collisions at the two endpoints: to and from
     */
    virtual bool is_straight_line_feasible(const StateT &from,
                                           const StateT &to,
                                           uint32_t step_size) const {
        for (uint32_t i = 1; i <= step_size; ++i) {
            double t = static_cast<double>(i) / step_size;
            StateT intermediate = interpolate(from, to, t);
            if (is_in_collision(intermediate)) return false;
        }
        return true;
    }
};

} // namespace rrt
#pragma once

#include "state_space.hpp"
#include <vector>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <memory>
#include <algorithm>
#include <random>
#include <optional>

namespace rrt {
struct PlannerConfig {
    double goal_bias;
    double goal_radius;

    /**
     * @brief scalar value for step size taken when extending tree
     */
    double step_size;

    /**
     * @brief Number of interpolation samples to check
     * for collision detection when extending tree
     */
    uint32_t interpolation_resolution;
};

template<typename StateT>
class RRTTree {
public:
    explicit RRTTree(std::shared_ptr<StateSpace<StateT>> state_space)
            : state_space(std::move(state_space)) {}

    virtual ~RRTTree() = default;

    /**
     * @brief adds a node into the tree
     * @return the index of the newly added node
     */
    virtual std::size_t add_node(StateT state, int parent_idx) = 0;

    virtual std::size_t nearest_neighbor(const StateT &state) const = 0;

    /**
     * @brief Finds and returns the path from root to the specified node
     * @return A vector of the states in the path starting from the root node.
     * @param node_idx Index of target node in the tree
     * @note This function performs O(L) state copies, where L is the length of
     *      the path.
     */
    virtual std::vector<StateT> get_path_from_root(std::size_t node_idx) const = 0;

    virtual const StateT& get_node(std::size_t idx) const = 0;

    virtual std::size_t size() const = 0;

    virtual void clear() = 0;

protected:
    std::shared_ptr<StateSpace<StateT>> state_space;
};

// shouldn't be visible to the outside
// used for generating random sample of goal bias
static thread_local std::mt19937 generator{std::random_device{}()};
static thread_local std::uniform_real_distribution<double> unif_01(0.0, 1.0);

template<typename StateT>
class Planner {
public:
    virtual ~Planner() = default;

    explicit Planner(std::shared_ptr<StateSpace<StateT>> state_space,
                     PlannerConfig config) :
        state_space(std::move(state_space)), config(config) { }

    /**
     * @brief User facing function for running the RRT search algorithm
     * @note If solve is invoked more than once, previous results will be cleared
     */
     virtual std::optional<std::vector<StateT>> solve(const StateT &start, const StateT &goal,
                        std::chrono::milliseconds timeout) final {
         tree = make_tree();
         tree->add_node(start, -1);
         return plan(start, goal, timeout);
     }

protected:
    /**
     * @brief Override function that runs the RRT search algorithm and updates the RRT tree
     * @param start start state
     * @param goal goal state
     * @param timeout the duration of the search
     * @return true if solution was found within timeout, false otherwise
     * @note This is a non-virtual interface - it should never be exposed with public visibility
     */
    virtual std::optional<std::vector<StateT>> plan(const StateT &start, const StateT &goal,
                      std::chrono::milliseconds timeout) = 0;

    /**
     * @brief Factory method for creating an RRT Tree for this planner
     */
    virtual std::unique_ptr<RRTTree<StateT>> make_tree() = 0;

    std::shared_ptr<StateSpace<StateT>> state_space;
    const PlannerConfig config;
    std::unique_ptr<RRTTree<StateT>> tree;

    /**
     * @brief Produces a random floating point number between 0 and 1
     * @info Utility function for realizing goal bias
     */
    double sample_01() {
        return unif_01(generator);
    }
};
} // namespace rrt

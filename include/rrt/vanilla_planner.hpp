#pragma once

#include "rrt/planner.hpp"
#include <random>
#include <chrono>

namespace rrt {
template<typename StateT>
class VanillaRRTTree : public RRTTree<StateT> {
public:
    explicit VanillaRRTTree(std::shared_ptr<StateSpace<StateT>> state_space)
            : RRTTree<StateT>(std::move(state_space)) {}

    std::size_t add_node(StateT state, int parent_idx) override {
        nodes.emplace_back(std::move(state));
        predecessors.push_back(parent_idx);
        return nodes.size() - 1;
    }

    std::size_t nearest_neighbor(const StateT &state) const override {
        // tree is never empty
        std::size_t result = 0;
        double min_dist = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < nodes.size(); ++i) {
            double dist = this->state_space->distance(nodes[i], state);
            if (dist < min_dist) {
                min_dist = dist;
                result = i;
            }
        }

        return result;
    }

    std::vector<StateT> get_path_from_root(std::size_t node_idx) const override {
        if (predecessors[node_idx] == -1) {
            return {nodes[node_idx]};
        }

        std::vector<StateT> path;
        int curr = static_cast<int>(node_idx);

        while (curr >= 0) {
            path.push_back(nodes[curr]);
            curr = predecessors[curr];
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    const StateT& get_node(std::size_t idx) const override {
        return nodes.at(idx);
    }

    std::size_t size() const override {
        return nodes.size();
    }

    void clear() override {
        nodes.clear();
        predecessors.clear();
    }

private:
    std::vector<StateT> nodes;
    std::vector<int> predecessors;
};

/**
 * @brief Single-threaded RRT planner using standard RRT algorithm
 */
template<typename StateT>
class VanillaPlanner : public Planner<StateT> {
public:
    explicit VanillaPlanner(
            std::shared_ptr<StateSpace<StateT>> state_space,
            PlannerConfig config)
            : Planner<StateT>(std::move(state_space), config) {}

protected:
    std::optional<std::vector<StateT>> plan(const StateT &start,
              const StateT &goal,
              std::chrono::milliseconds timeout) override;

    std::unique_ptr<RRTTree<StateT>> make_tree() override {
        return std::make_unique<VanillaRRTTree<StateT>>(this->state_space);
    }
};

template<typename StateT>
std::optional<std::vector<StateT>> VanillaPlanner<StateT>::plan(const StateT &start, const StateT &goal,
                                                                std::chrono::milliseconds timeout) {
    const auto stop_time = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < stop_time) {
        // sample from [0,1]
        StateT sample =
                (this->sample_01() < this->config.goal_bias)
                ? goal
                : this->state_space->uniform_sample();

        std::size_t nearest_neighbor_idx = this->tree->nearest_neighbor(sample);
        const StateT& nearest_neigbor = this->tree->get_node(nearest_neighbor_idx);
        StateT new_node = this->state_space->branch_extend(nearest_neigbor,
                                                           sample,
                                                           this->config.step_size);

        if (this->state_space->is_in_collision(new_node) || !this->state_space->is_straight_line_feasible(nearest_neigbor, new_node, this->config.interpolation_resolution)) {
            continue;
        }

        // check if we have reached goal
        if (this->state_space->distance(new_node, goal) < this->config.goal_radius) {
            if (this->state_space->is_straight_line_feasible(new_node, goal, this->config.interpolation_resolution)) {
                std::size_t node_idx = this->tree->add_node(std::move(new_node), nearest_neighbor_idx);
                size_t  goal_idx = this->tree->add_node(this->state_space->clone(goal), node_idx);
                return this->tree->get_path_from_root(goal_idx);
            }
        } else {
            this->tree->add_node(std::move(new_node), nearest_neighbor_idx);
        }
    }
    return std::nullopt;
}
}// namespace rrt

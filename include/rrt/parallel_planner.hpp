#pragma once

#include "rrt/planner.hpp"
#include <random>
#include <chrono>
#include <shared_mutex>
#include <thread>
#include <condition_variable>
#include <algorithm>
#include <future>

namespace rrt {
template <typename StateT>

/**
 * RRT Tree implementation that is thread-safe
 * @note Assumption: StateT is default constructible (for resize)
 */
class ParallelRRTTree : public RRTTree<StateT> {
public:
    explicit ParallelRRTTree(std::shared_ptr<StateSpace<StateT>> state_space, std::size_t reserve_hint = 1024)
            : RRTTree<StateT>(std::move(state_space)), id_counter(0), written_idx(0) {
        nodes.resize(std::max(reserve_hint, std::size_t{1024}));
        predecessors.resize(std::max(reserve_hint, std::size_t{1024}));
    }

    std::size_t add_node(StateT state, int parent_idx) override {
        // grab an id to insert to
        const std::size_t idx = id_counter.fetch_add(1, std::memory_order_relaxed);

        {
            // grab a read lock first
            std::shared_lock<std::shared_mutex> read_lock(rwlock);

            if (idx >= nodes.size()) {
                read_lock.unlock();

                if (idx == nodes.size()) {
                    // if i am in charge of resizing, resize by acquiring write lock
                    std::unique_lock<std::shared_mutex> write_lock(rwlock);
                    // more aggresive resized
                    nodes.resize(nodes.size() * 4);
                    predecessors.resize(predecessors.size() * 4);

                    {
                        std::lock_guard<std::mutex> lk(resize_mtx);
                        resize_cnt++;
                    }

                    resize_cv.notify_all();
                } else {
                    // else we sleep on the resize condvar
                    std::unique_lock<std::mutex> lock(resize_mtx);
                    std::size_t curr_cnt = resize_cnt;
                    resize_cv.wait(lock, [&] { return resize_cnt > curr_cnt; });
                }

                std::shared_lock<std::shared_mutex> read_lock(rwlock);
                nodes[idx] = state;
                predecessors[idx] = parent_idx;
            } else {
                // holding read lock
                nodes[idx] = state;
                predecessors[idx] = parent_idx;
            }
        }

        // try to push written_idx to be >= idx
        while (true) {
            std::size_t expected = idx;
            if (written_idx.compare_exchange_weak(
                    expected,
                    idx + 1,
                    std::memory_order_release,
                    std::memory_order_relaxed)) {
                return idx;
            }
            std::this_thread::yield();
        }
    }

    std::size_t nearest_neighbor(const StateT &state) const override {
        // tree is never empty
        std::shared_lock<std::shared_mutex> lock(rwlock);

        const std::size_t n = size();
        std::size_t result = 0;
        double min_dist = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < n; ++i) {
            double dist = this->state_space->distance(nodes[i], state);
            if (dist < min_dist) {
                min_dist = dist;
                result = i;
            }
        }

        return result;
    }

    std::vector<StateT> get_path_from_root(std::size_t node_idx) const override {
        std::shared_lock<std::shared_mutex> lock(rwlock);
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
        std::shared_lock<std::shared_mutex> lock(rwlock);
        return nodes.at(idx);
    }

    std::size_t size() const override {
        // note, this is user facing size which we expose as all the completed writes
        return written_idx.load(std::memory_order_acquire);
    }

    /**
     * @warning Not thread safe
     */
    void clear() override {
        nodes.clear();
        predecessors.clear();
        id_counter = 0;
        written_idx = 0;
        resize_cnt = 0;
    }

private:
    std::vector<StateT> nodes;
    std::vector<int> predecessors;
    mutable std::shared_mutex rwlock;
    std::atomic<std::size_t> id_counter;
    // everything from [0, written_idx) should be completely written
    std::atomic<std::size_t> written_idx;

    // resize conditional variable
    std::condition_variable resize_cv;
    std::mutex resize_mtx;
    std::size_t resize_cnt = 0;
};

template<typename StateT>
class ParallelPlanner : public Planner<StateT> {
public:
    /**
     * Parallel RRT tree expansion with multithreading
     * @param workers number of threads for the task
     * @param reserve_hint default reserved number of allocations for RRT tree vector
     *      to reduce number of resizes
     */
    explicit ParallelPlanner(
            std::shared_ptr<StateSpace<StateT>> state_space,
            PlannerConfig config, uint8_t workers, std::size_t reserve_hint)
            : Planner<StateT>(std::move(state_space), config), workers(workers), reserve_hint(reserve_hint) {}

protected:
    std::optional<std::vector<StateT>> plan(const StateT &start,
              const StateT &goal,
              std::chrono::milliseconds timeout) override;

    std::unique_ptr<RRTTree<StateT>> make_tree() override {
        return std::make_unique<ParallelRRTTree<StateT>>(this->state_space, this->reserve_hint);
    }

private:
    uint8_t workers;
    std::size_t reserve_hint;
};

template<typename StateT>
std::optional<std::vector<StateT>> ParallelPlanner<StateT>::plan(const StateT &start, const StateT &goal,
                                                                std::chrono::milliseconds timeout) {
    const auto stop_time = std::chrono::steady_clock::now() + timeout;

    std::atomic<bool> done{false};
    std::atomic<int> global_goal_idx{-1};
    std::vector<std::thread> threads;
    threads.reserve(this->workers);

    for (int i = 0; i < this->workers; ++i) {
        threads.emplace_back([&]() {
            while (std::chrono::steady_clock::now() < stop_time && !done.load(std::memory_order_acquire)) {
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
                        int goal_idx = this->tree->add_node(this->state_space->clone(goal), node_idx);
                        done.store(true, std::memory_order_release);
                        global_goal_idx.store(goal_idx, std::memory_order_release);
                        return;
                    }
                } else {
                    this->tree->add_node(std::move(new_node), nearest_neighbor_idx);
                }
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    int result = global_goal_idx.load(std::memory_order_acquire);
    if (result != -1) {
        return this->tree->get_path_from_root(result);
    } else {
        return std::nullopt;
    }
}
}// namespace rrt

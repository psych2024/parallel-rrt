#include "../euclidean_2d_space.hpp"
#include <nanobench.h>
#include <iostream>

#ifdef USE_PARALLEL_PLANNER
#include "rrt/parallel_planner.hpp"
#define PLANNER_NAME "ParallelPlanner"
#else
#include "rrt/vanilla_planner.hpp"
#define PLANNER_NAME "VanillaPlanner"
#endif

using namespace rrt;
using namespace std;
using namespace std::chrono_literals;
using namespace ankerl::nanobench;

#ifndef NUM_WORKERS
#define NUM_WORKERS 4
#endif

#ifndef RESERVE_HINT
#define RESERVE_HINT 2048
#endif

#ifndef TIMEOUT_MS
#define TIMEOUT_MS 10000
#endif

int main() {
    auto config = PlannerConfig{0.2, 0.025, 0.025, 500};

    double max_x = 1.3784;
    double max_y = 1.0;
    auto space = make_shared<Euclidean2DSpace>(0, max_x, 0, max_y);

    space->add_obstacle(Circle{
        0.25,
        Point2D{max_x / 2, max_y / 2}
    });

    space->add_obstacle(Circle{
        0.15,
        Point2D{max_x * 0.3, max_y * 0.75}
    });

    space->add_obstacle(Circle{
        0.15,
        Point2D{max_x * 0.7, max_y * 0.75}
    });

    space->add_obstacle(Circle{
        0.15,
        Point2D{max_x * 0.3, max_y * 0.25}
    });

    space->add_obstacle(Circle{
        0.15,
        Point2D{max_x * 0.7, max_y * 0.25}
    });

    space->add_obstacle(Circle{
        0.12,
        Point2D{max_x * 0.5, max_y * 0.85}
    });

    space->add_obstacle(Circle{
        0.12,
        Point2D{max_x * 0.5, max_y * 0.15}
    });

    space->add_obstacle(Circle{
        0.1,
        Point2D{max_x * 0.15, max_y * 0.85}
    });

    space->add_obstacle(Circle{
        0.1,
        Point2D{max_x * 0.85, max_y * 0.15}
    });

    Point2D start = {0.05, 0.5};
    Point2D goal = {max_x - 0.05, 0.5};

#ifdef USE_PARALLEL_PLANNER
    ParallelPlanner<Point2D> planner(space, config, NUM_WORKERS, RESERVE_HINT);
#else
    VanillaPlanner<Point2D> planner(space, config);
#endif

    // run once first to make sure a solution is actually found
    auto result = planner.solve(start, goal, chrono::milliseconds(TIMEOUT_MS));
    if (result.has_value()) {
        cout << "solution found" << endl;
    } else {
        cout << "solution not found" << endl;
    }

    Bench bench;
    bench.minEpochIterations(100);
    bench.run("rrt", [&] {
        auto result = planner.solve(start, goal, chrono::milliseconds(TIMEOUT_MS));
        doNotOptimizeAway(result);
    });

    return 0;
}

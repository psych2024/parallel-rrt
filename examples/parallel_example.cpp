#include "euclidean_2d_space.hpp"
#include <iostream>
#include "rrt/parallel_planner.hpp"

using namespace rrt;
using namespace std;

using namespace std::chrono_literals;

/**
 * @brief Example similar to RRT question on PA-U4
 */
int main() {
    auto config = PlannerConfig {0.2, 0.025, 0.05, 500};

    double max_x = 1.3784;
    double max_y = 1.0;
    auto space = make_shared<Euclidean2DSpace>(0, max_x, 0, max_y);

    space->add_obstacle(Circle{
        0.4,
        Point2D {max_x / 2, max_y / 2}
    });

    Point2D start = {0.1, 0.5};
    Point2D goal = {1.278, 0.5};
    ParallelPlanner<Point2D> planner(space, config, 2, 2048);
    auto result = planner.solve(start, goal, 3000ms);
    if (result.has_value()) {
        cout << "solution found!" << endl;
        for (auto &p : result.value()) {
            cout << p.x << " " << p.y << endl;
        }
    } else {
        cout << "solution not found!";
    }
}
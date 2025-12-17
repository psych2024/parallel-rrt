#pragma once

#include "rrt/state_space.hpp"
#include <random>
#include <cmath>

using namespace rrt;

struct Point2D {
    double x;
    double y;
};

struct Circle {
    double radius;
    Point2D position;
};

/**
 * @brief Concrete 2D Euclidean state space for RRT benchmarking
 * Implemented to demonstrate RRT just like in class
 * As part of demo, can create circle obstacles in our state space
 */
class Euclidean2DSpace : public StateSpace<Point2D> {
public:
    Euclidean2DSpace(double min_x, double max_x,
                     double min_y, double max_y);

    ~Euclidean2DSpace() override = default;

    double distance(const Point2D &s1, const Point2D &s2) const override;

    Point2D clone(const Point2D& s) const override;

    Point2D branch_extend(const Point2D& source,
                   const Point2D& target,
                   double step_size) const override;

    Point2D interpolate(const Point2D& from,
                        const Point2D& to,
                        double t) const override;

    Point2D uniform_sample() const override;

    bool is_in_collision(const Point2D& state) const override;

    void add_obstacle(const Circle& circle);

private:
    double min_x;
    double max_x;
    double min_y;
    double max_y;

    std::vector<Circle> obstacles;
};
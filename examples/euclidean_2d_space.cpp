#include "euclidean_2d_space.hpp"
#include <cmath>
#include <random>

Euclidean2DSpace::Euclidean2DSpace(double min_x, double max_x,
                                   double min_y, double max_y)
        : min_x(min_x), max_x(max_x),
          min_y(min_y), max_y(max_y) {}

Point2D Euclidean2DSpace::clone(const Point2D &s) const {
    return Point2D{s.x, s.y};
}

double Euclidean2DSpace::distance(const Point2D &s1, const Point2D &s2) const {
    double dx = s1.x - s2.x;
    double dy = s1.y - s2.y;
    return sqrt(dx * dx + dy * dy);
}

Point2D
Euclidean2DSpace::branch_extend(const Point2D &source, const Point2D &target,
                         double step_size) const {
    double dx = target.x - source.x;
    double dy = target.y - source.y;

    double norm = sqrt(dx * dx + dy * dy);
    if (norm <= step_size) {
        return target;
    }

    dx /= norm;
    dy /= norm;
    return Point2D {dx * step_size + source.x, dy * step_size + source.y};
}

Point2D
Euclidean2DSpace::interpolate(const Point2D &from, const Point2D &to,
                              double t) const {
    return Point2D{
            from.x + t * (to.x - from.x),
            from.y + t * (to.y - from.y)
    };
}

Point2D Euclidean2DSpace::uniform_sample() const {
    static thread_local std::uniform_real_distribution<double> x_dist(min_x, max_x);
    static thread_local std::uniform_real_distribution<double> y_dist(min_y, max_y);

    static thread_local std::mt19937 generator{std::random_device{}()};
    return {x_dist(generator), y_dist(generator)};
}

bool Euclidean2DSpace::is_in_collision(const Point2D &state) const {
    for (auto &circle : this->obstacles) {
        if (distance(state, circle.position) <= circle.radius) return true;
    }
    return false;
}

void Euclidean2DSpace::add_obstacle(const Circle &circle) {
    this->obstacles.push_back(circle);
}

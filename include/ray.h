#pragma once

#include <Eigen/Core>

struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;

    Ray() {}
    Ray(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction)
        : origin(origin), direction(direction.normalized()) {}

    Eigen::Vector3f operator()(float t) const { return origin + t * direction; }
}; 
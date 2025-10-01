#pragma once

#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct SplatScene {
    // Core attributes
    std::vector<Eigen::Vector3f> positions;        // (x, y, z)
    std::vector<Eigen::Vector3f> scales;           // Gaussian radii (σx, σy, σz)
    std::vector<Eigen::Quaternionf> rotations;     // Orientation
    std::vector<float> opacity;

    // SH color coefficients: 16 RGB triplets per vertex
    std::vector<std::array<Eigen::Vector3f, 16>> sh_coeffs;

    // Optional precomputations
    std::vector<Eigen::Matrix3f> covariances;
    std::vector<float> bounding_radii;

    size_t size() const { return positions.size(); }

    void reserve(size_t n) {
        positions.reserve(n);
        scales.reserve(n);
        rotations.reserve(n);
        opacity.reserve(n);
        sh_coeffs.reserve(n);
        covariances.reserve(n);
        bounding_radii.reserve(n);
    }
};

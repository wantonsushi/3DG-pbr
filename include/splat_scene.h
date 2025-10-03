#pragma once

#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <limits>
#include <memory>

#include "ray.h"
#include "rng.h"

#include <iostream>

struct SplatScene {
    const float R = 3.0f;

    std::vector<Eigen::Vector3f> prim_bbox_min;
    std::vector<Eigen::Vector3f> prim_bbox_max;

    // Core attributes
    std::vector<Eigen::Vector3f> positions;        // (x, y, z)
    std::vector<Eigen::Vector3f> scales;           // Gaussian radii (σx, σy, σz)
    std::vector<Eigen::Quaternionf> rotations;     // Orientation
    std::vector<float> opacity;

    // SH color coefficients: 16 RGB triplets per vertex
    std::vector<std::array<Eigen::Vector3f, 16>> sh_coeffs;

    // Optional precomputations
    std::vector<Eigen::Matrix3f> covariances;
    std::vector<Eigen::Matrix3f> inv_covariances; // precompute inverse
    std::vector<float> bounding_radii;

    size_t size() const { return positions.size(); }

    void reserve(size_t n) {
        positions.reserve(n);
        scales.reserve(n);
        rotations.reserve(n);
        opacity.reserve(n);
        sh_coeffs.reserve(n);
        covariances.reserve(n);
        inv_covariances.reserve(n);
        bounding_radii.reserve(n);
    }

    // Precompute covariances and their inverses
    void precompute() {
        size_t N = positions.size();
        covariances.resize(N);
        inv_covariances.resize(N);
        bounding_radii.resize(N);

        prim_bbox_min.resize(N);
        prim_bbox_max.resize(N);

        for (size_t i = 0; i < N; ++i) {
            Eigen::Matrix3f S = Eigen::Matrix3f::Identity();
            S(0,0) = scales[i].x();
            S(1,1) = scales[i].y();
            S(2,2) = scales[i].z();

            Eigen::Matrix3f Rm = rotations[i].normalized().toRotationMatrix();
            Eigen::Matrix3f M = Rm * S;
            covariances[i] = M * M.transpose();
            inv_covariances[i] = covariances[i].inverse();

            // 3σ bounding-sphere (kept for compatibility / fallback)
            bounding_radii[i] = 3.0f * scales[i].norm();

            // --- compute tight axis-aligned AABB for the 3σ ellipsoid ---
            // eigen-decomposition of covariance (symmetric, so use SelfAdjointEigenSolver)
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(covariances[i]);
            Eigen::Matrix3f eigvecs = es.eigenvectors();               // columns are eigenvectors
            Eigen::Vector3f eigvals = es.eigenvalues().cwiseMax(0.0f); // ensure non-negative numerically

            // axes lengths (3 * sqrt(lambda))
            Eigen::Vector3f axes = 3.0f * eigvals.cwiseSqrt(); // a_j

            // extents along world X/Y/Z: ext = sum_j abs(u_j) * a_j
            // where u_j is eigenvector column j. In Eigen: abs(eigvecs) * axes
            Eigen::Matrix3f absU = eigvecs.cwiseAbs();
            Eigen::Vector3f ext = absU * axes; // 3x3 * 3x1 -> 3x1 extents

            prim_bbox_min[i] = positions[i] - ext;
            prim_bbox_max[i] = positions[i] + ext;
        }

        buildBVH();
    }

    // ---------------- BVH Structures ----------------
    struct BVHNode {
        Eigen::Vector3f bbox_min;
        Eigen::Vector3f bbox_max;
        int left = -1;
        int right = -1;
        int start = -1;  // index range for leaf (start inclusive)
        int end = -1;    // index range for leaf (exclusive)
    };

    std::vector<BVHNode> bvh_nodes;
    std::vector<int> bvh_indices; // indices into positions/scales

    void buildBVH() {
        bvh_indices.resize(size());
        for (size_t i = 0; i < size(); ++i) bvh_indices[i] = int(i);

        bvh_nodes.clear();
        buildBVHRecursive(0, int(size()));
    }

    int buildBVHRecursive(int start, int end) {
        BVHNode node;
        node.start = start;
        node.end = end;

        // compute bounding box
        Eigen::Vector3f min_pt(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        Eigen::Vector3f max_pt(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

        // in buildBVHRecursive
        for (int i = start; i < end; ++i) {
            int idx = bvh_indices[i];
            min_pt = min_pt.cwiseMin(prim_bbox_min[idx]);
            max_pt = max_pt.cwiseMax(prim_bbox_max[idx]);
        }

        node.bbox_min = min_pt;
        node.bbox_max = max_pt;

        int node_idx = int(bvh_nodes.size());
        bvh_nodes.push_back(node);

        int count = end - start;
        if (count <= 1) return node_idx; // leaf

        // choose axis with largest extent
        Eigen::Vector3f extent = max_pt - min_pt;
        int axis;
        extent.maxCoeff(&axis);

        // sort along axis
        std::sort(bvh_indices.begin() + start, bvh_indices.begin() + end,
                  [&](int a, int b) { return positions[a][axis] < positions[b][axis]; });

        int mid = start + count / 2;

        int left_idx = buildBVHRecursive(start, mid);
        int right_idx = buildBVHRecursive(mid, end);

        bvh_nodes[node_idx].left = left_idx;
        bvh_nodes[node_idx].right = right_idx;
        return node_idx;
    }

    // ---------------- Analytic Gaussian Intersection ----------------
    bool intersectGaussian(int idx, const Ray& ray, float& t_mid, PCG32& rng) const {
        const Eigen::Vector3f& mean = positions[idx];
        const Eigen::Matrix3f& inv_cov = inv_covariances[idx];

        // Shift ray origin to Gaussian local space
        Eigen::Vector3f p = ray.origin - mean;

        Eigen::Vector3f Md = inv_cov * ray.direction;
        Eigen::Vector3f Mp = inv_cov * p;

        float A = ray.direction.dot(Md);
        float B = 2.0f * p.dot(Md);
        float C = p.dot(Mp) - R*R;

        float disc = B*B - 4.0f*A*C;
        if (disc < 0.0f) return false;

        float sqrtD = std::sqrt(disc);
        float t0 = (-B - sqrtD) / (2.0f*A);
        float t1 = (-B + sqrtD) / (2.0f*A);
        if (t0 > t1) std::swap(t0, t1);
        if (t1 < 0.0f) return false;
        t0 = std::max(t0, 0.0f);

        t_mid = 0.5f * (t0 + t1);

        // stochastic acceptance
        float g_val = std::exp(-0.5f * (ray.origin + t_mid*ray.direction - mean).transpose() * inv_cov * (ray.origin + t_mid*ray.direction - mean));
        float p_accept = opacity[idx] * g_val;

        return rng.uniform() < p_accept;
    }

    // ---------------- BVH Ray Query ----------------
    bool intersectBVHStochastic(const Ray& ray, float& t_hit, int& hit_idx, float t_max, int node_idx, PCG32& rng) const {
        if (node_idx < 0 || node_idx >= int(bvh_nodes.size())) return false;
        const BVHNode& node = bvh_nodes[node_idx];

        // AABB intersection
        Eigen::Vector3f inv_dir = ray.direction.cwiseInverse();
        Eigen::Vector3f t0s = (node.bbox_min - ray.origin).cwiseProduct(inv_dir);
        Eigen::Vector3f t1s = (node.bbox_max - ray.origin).cwiseProduct(inv_dir);
        float tmin = t0s.minCoeff();
        float tmax_box = t1s.maxCoeff();
        if (tmax_box < 0.0f || tmin > t_max) return false;

        // leaf
        if (node.left == -1 && node.right == -1) {
            bool hit = false;
            for (int i = node.start; i < node.end; ++i) {
                int idx = bvh_indices[i];
                float t_mid;
                if (intersectGaussian(idx, ray, t_mid, rng) && t_mid < t_max) {
                    t_max = t_mid;
                    t_hit = t_mid;
                    hit_idx = idx;
                    hit = true;
                }
            }
            return hit;
        }

        bool hit_left = intersectBVHStochastic(ray, t_hit, hit_idx, t_max, node.left, rng);
        bool hit_right = intersectBVHStochastic(ray, t_hit, hit_idx, t_max, node.right, rng);
        return hit_left || hit_right;
    }
};

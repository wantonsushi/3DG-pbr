#pragma once
#include <Eigen/Core>
#include <GL/gl.h>

struct SimpleSphere {
    Eigen::Vector3f position;
    float radius;
    Eigen::Vector3f color;
};

class Rasterizer {
public:
    Rasterizer() = default;

    void draw_spheres();

private:
    std::vector<SimpleSphere> spheres = {
        {{0, 0, 0}, 0.5f, {1.0f, 0.0f, 0.0f}},
        {{1.0f, 0.5f, -1.0f}, 0.3f, {0.0f, 1.0f, 0.0f}},
        {{-1.0f, -0.5f, -0.5f}, 0.4f, {0.0f, 0.0f, 1.0f}}
    };
};

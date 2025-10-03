#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <GL/gl.h>

#include "common.h"
#include "splat_scene.h"

struct SimpleSphere {
    Eigen::Vector3f position;
    float radius;
    Eigen::Vector3f color;
};

struct GData {
    Eigen::Matrix3f Sigma_world;
    Eigen::Vector2f px;
    float view_z; // for sorting (eye-space z)
    Eigen::Vector3f rgb; // evaluated SH
    float opacity;
};

class Rasterizer {
public:
    Rasterizer(int w, int h);
    ~Rasterizer();

    // set scene (pretrained SplatScene)
    void setScene(const SplatScene* s) { scene = s; }

    // full splat pipeline
    void draw_splats();
private:
    // Helpers
    Eigen::Matrix4f glGetFloatMatrix(GLenum pname) const;
    Eigen::Vector2f worldToPixel(const Eigen::Vector3f& p_world, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, int W, int H) const;
    Eigen::Matrix2f computeCov2D(const Eigen::Vector3f& mean_world, const Eigen::Matrix3f& Sigma_world,
                                    const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view,
                                    int W, int H) const;
    Eigen::Matrix3f quatToRotation(const Eigen::Quaternionf& q) const;

    // draw fullscreen quad with texture
    void uploadAndDrawBuffer(int W, int H, const std::vector<float>& buffer);

    const SplatScene* scene = nullptr;

    // GL texture for final buffer
    GLuint fb_tex = 0;

    std::vector<float> out_color;
    std::vector<float> final_T;
    std::vector<uint32_t> n_contrib;
    std::vector<float> invdepth;
    std::vector<size_t> order;
    std::vector<GData> gdat;
};

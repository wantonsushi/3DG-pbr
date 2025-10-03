#pragma once
#include <vector>
#include <memory>
#include <Eigen/Core>
#include <GL/gl.h>

#include "common.h"
#include "splat_scene.h"
#include "camera.h"
#include "rng.h"

class RayTracer {
public:
    RayTracer(int w, int h, int spp = 1);
    ~RayTracer();

    void setScene(const SplatScene* s) { scene = s; }
    void setCamera(const Camera* c) {camera = c; }

    // Render a frame into buffer and push to OpenGL
    void render_frame(int fi);

private:
    int W, H, spp;
    const SplatScene* scene = nullptr;
    const Camera* camera = nullptr;

    // Framebuffer storage (RGB float)
    std::vector<float> framebuffer;
    GLuint fb_tex = 0;
};

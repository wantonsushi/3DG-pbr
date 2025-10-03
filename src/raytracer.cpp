#include "raytracer.h"
#include <iostream>

#include "rng.h"
#include "common.h"

#include <omp.h>

RayTracer::RayTracer(int w, int h, int spp) : W(w), H(h), spp(spp) {
    framebuffer.resize(W * H * 3, 0.0f);

    glGenTextures(1, &fb_tex);
    glBindTexture(GL_TEXTURE_2D, fb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

RayTracer::~RayTracer() {
    if (fb_tex) {
        glDeleteTextures(1, &fb_tex);
    }
}

void RayTracer::render_frame(int fi) {
    if (!scene || !camera) return;

    // Loop over pixels
    #pragma omp parallel for collapse(2) schedule(dynamic,1)
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            Eigen::Vector3f accumulated_color(0,0,0);
            for(int si =0; si < spp; si++) {
                PCG32 rng(derive_seed(x, y, si, fi), 1);
                float jitter_x = rng.uniform();
                float jitter_y = rng.uniform();
                float px = (x + jitter_x) / float(W);
                float py = (y + jitter_y) / float(H);

                Ray ray = camera->generate_ray(px, py);

                float t_hit;
                int hit_idx;
                Eigen::Vector3f color(0,0,0);
                
                if (scene->intersectBVHStochastic(ray, t_hit, hit_idx,
                                                std::numeric_limits<float>::max(),
                                                0, rng))
                {
                    Eigen::Vector3f dir = -ray.direction.normalized();
                    const Eigen::Vector3f* coeffs = scene->sh_coeffs[hit_idx].data();
                    color = evalSH(3, dir, coeffs);
                }

                accumulated_color += color;
            }

            accumulated_color /= float(spp);

            int idx = (y * W + x) * 3;
            framebuffer[idx+0] = accumulated_color.x();
            framebuffer[idx+1] = accumulated_color.y();
            framebuffer[idx+2] = accumulated_color.z();
        }
    }

    // Upload + draw
    uploadAndDrawBuffer(fb_tex, W, H, framebuffer);
}
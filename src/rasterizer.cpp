#include "rasterizer.h"

#include <algorithm>
#include <cmath>
#include <iostream>

Rasterizer::Rasterizer()
{
    glGenTextures(1, &fb_tex);
    glBindTexture(GL_TEXTURE_2D, fb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

Rasterizer::~Rasterizer()
{
    if (fb_tex) glDeleteTextures(1, &fb_tex);
}

Eigen::Matrix4f Rasterizer::glGetFloatMatrix(GLenum pname) const
{
    GLfloat m[16];
    glGetFloatv(pname, m);
    Eigen::Matrix4f M;
    // OpenGL returns column-major flattened as m[row + col*4]
    // so row = i % 4, col = i / 4
    for (int i = 0; i < 16; ++i) {
        int row = i % 4;
        int col = i / 4;
        M(row, col) = m[i];
    }
    return M;
}

Eigen::Vector2f Rasterizer::worldToPixel(const Eigen::Vector3f& p_world, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, int W, int H) const
{
    Eigen::Vector4f ph = proj * view * Eigen::Vector4f(p_world.x(), p_world.y(), p_world.z(), 1.0f);
    // homogeneous divide
    if (std::abs(ph.w()) < 1e-9f) return Eigen::Vector2f(-1e9f, -1e9f);
    Eigen::Vector3f ndc(ph.x() / ph.w(), ph.y() / ph.w(), ph.z() / ph.w());
    // convert NDC to pixel coords
    float px = (ndc.x() * 0.5f + 0.5f) * float(W);
    float py = (1.0f - (ndc.y() * 0.5f + 0.5f)) * float(H); // flip y ?
    return { px, py };
}

// see https://github.com/graphdeco-inria/diff-gaussian-rasterization/blob/9c5c2028f6fbee2be239bc4c9421ff894fe4fbe0/cuda_rasterizer/forward.cu
Eigen::Matrix2f Rasterizer::computeCov2D(
    const Eigen::Vector3f& mean_world,
    const Eigen::Matrix3f& Sigma_world,
    const Eigen::Matrix4f& proj,
    const Eigen::Matrix4f& view,
    int W, int H) const
{
    Eigen::Vector4f p4(mean_world.x(), mean_world.y(), mean_world.z(), 1.0f);
    Eigen::Vector4f t4 = view * p4;
    float tx = t4.x();
    float ty = t4.y();
    float tz = t4.z();

    float a00 = proj(0,0);
    float a11 = proj(1,1);

    const float tan_fovx = 1.0f / a00;
    const float tan_fovy = 1.0f / a11;

    const float limx = 1.3f * tan_fovx;
    const float limy = 1.3f * tan_fovy;

    float txtz = tx / tz;
    float tytz = ty / tz;
    float clamped_x = std::min(limx, std::max(-limx, txtz));
    float clamped_y = std::min(limy, std::max(-limy, tytz));

    tx = clamped_x * tz;
    ty = clamped_y * tz;

    // analytic Jacobian J (3x3) of projection
    float focal_x = 0.5f * float(W) * a00; 
    float focal_y = 0.5f * float(H) * a11;

    Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
    J(0,0) = focal_x / tz;
    J(0,2) = -(focal_x * tx) / (tz * tz);
    J(1,1) = focal_y / tz;
    J(1,2) = -(focal_y * ty) / (tz * tz);

    Eigen::Matrix3f Wmat = view.block<3,3>(0,0);

    Eigen::Matrix3f T = Wmat * J;

    Eigen::Matrix3f tmp = Sigma_world * T;
    Eigen::Matrix3f cov3 = T.transpose() * tmp;

    Eigen::Matrix2f cov2D;
    cov2D(0,0) = cov3(0,0);
    cov2D(0,1) = cov3(0,1);
    cov2D(1,0) = cov3(1,0);
    cov2D(1,1) = cov3(1,1);

    return cov2D;
}

Eigen::Matrix3f Rasterizer::quatToRotation(const Eigen::Quaternionf& q_in) const
{
    Eigen::Quaternionf q = q_in.normalized();
    return q.toRotationMatrix();
}

void Rasterizer::uploadAndDrawBuffer(int W, int H, const std::vector<float>& buffer)
{
    glBindTexture(GL_TEXTURE_2D, fb_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, W, H, 0, GL_RGB, GL_FLOAT, buffer.data());

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, fb_tex);

    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex2f(-1, -1);
    glTexCoord2f(1, 0); glVertex2f( 1, -1);
    glTexCoord2f(1, 1); glVertex2f( 1,  1);
    glTexCoord2f(0, 1); glVertex2f(-1,  1);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);

    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void Rasterizer::draw_splats()
{
    // get viewport size
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    int W = vp[2];
    int H = vp[3];

    Eigen::Matrix4f proj = glGetFloatMatrix(GL_PROJECTION_MATRIX);
    Eigen::Matrix4f view = glGetFloatMatrix(GL_MODELVIEW_MATRIX);

    Eigen::Matrix4f invView = view.inverse();
    Eigen::Vector3f cam_pos(invView(0,3), invView(1,3), invView(2,3));

    size_t P = scene->size();
    if (P == 0) return;

    // prep buffers
    std::vector<float> out_color(W * H * 3, 0.0f);
    std::vector<float> final_T(W * H, 1.0f);
    std::vector<uint32_t> n_contrib(W * H, 0);
    std::vector<float> invdepth(W * H, 0.0f);

    struct GData {
        Eigen::Matrix3f Sigma_world;
        Eigen::Vector2f px;
        float view_z; // for sorting (eye-space z)
        Eigen::Vector3f rgb; // evaluated SH
        float opacity;
    };

    std::vector<GData> gdat(P);

    for (size_t i = 0; i < P; ++i) {
        Eigen::Vector3f pos = scene->positions[i];

        // get world covariance
        Eigen::Matrix3f Sigma;
        if (!scene->covariances.empty() && scene->covariances.size() == P) {
            Sigma = scene->covariances[i];
        } else {
            Eigen::Matrix3f S = Eigen::Matrix3f::Identity();
            S(0,0) = scene->scales[i].x();
            S(1,1) = scene->scales[i].y();
            S(2,2) = scene->scales[i].z();
            Eigen::Matrix3f R = quatToRotation(scene->rotations[i]);
            Eigen::Matrix3f M = R * S;
            Sigma = M * M.transpose();
        }

        Eigen::Vector2f px = worldToPixel(pos, proj, view, W, H);

        Eigen::Vector4f p_view4 = view * Eigen::Vector4f(pos.x(), pos.y(), pos.z(), 1.0f);
        float view_z = p_view4.z();

        // evaluate SH
        Eigen::Vector3f rgb = evalSH(3, (cam_pos - pos).normalized(), scene->sh_coeffs[i].data());

        gdat[i] = { Sigma, px, view_z, rgb, scene->opacity[i] };
    }

    // sort indices by view_z ascending
    std::vector<size_t> order(P);
    for (size_t i = 0; i < P; ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return gdat[a].view_z < gdat[b].view_z;
    });

    // main loop
    const float h_var = 0.3f; // same as ref
    for (size_t oi = 0; oi < P; ++oi) {
        size_t i = order[oi];
        const GData& gd = gdat[i];

        // get image-space covariance
        Eigen::Matrix2f cov2D = computeCov2D(scene->positions[i], gd.Sigma_world, proj, view, W, H);

        cov2D(0,0) += h_var;
        cov2D(1,1) += h_var;

        // determinant and inverse
        float det = cov2D(0,0) * cov2D(1,1) - cov2D(0,1) * cov2D(1,0);
        if (std::abs(det) < 1e-9f) continue;
        Eigen::Matrix2f invCov = cov2D.inverse();

        // compute eigen-based radius (3 sigma)
        float mid = 0.5f * (cov2D(0,0) + cov2D(1,1));
        float disc = std::max(0.1f, mid * mid - det);
        float lambda1 = mid + std::sqrt(disc);
        float lambda2 = mid - std::sqrt(disc);
        float maxLam = std::max(lambda1, lambda2);
        float my_radius = std::ceil(3.0f * std::sqrt(std::max(0.0f, maxLam)));

        // bounding rectangle in pixel space
        int minx = std::max(0, int(std::floor(gd.px.x() - my_radius)));
        int maxx = std::min(W - 1, int(std::ceil (gd.px.x() + my_radius)));
        int miny = std::max(0, int(std::floor(gd.px.y() - my_radius)));
        int maxy = std::min(H - 1, int(std::ceil (gd.px.y() + my_radius)));

        for (int y = miny; y <= maxy; ++y) {
            for (int x = minx; x <= maxx; ++x) {
                int pix_id = y * W + x;
                float dx = (float(x) + 0.5f) - gd.px.x();
                float dy = (float(y) + 0.5f) - gd.px.y();

                float q = invCov(0,0) * dx*dx + 2.0f * invCov(0,1) * dx*dy + invCov(1,1) * dy*dy;
                float power = -0.5f * q;

                // skip if outside support
                if (power < -30.0f) continue;

                // compute alpha
                float raw = std::exp(power);
                float alpha = std::min(0.99f, gd.opacity * raw);
                if (alpha < 1.0f / 255.0f) continue;

                float T = final_T[pix_id];
                float contribWeight = alpha * T;
                if (contribWeight <= 0.0f) continue;

                out_color[3 * pix_id + 0] += gd.rgb.x() * contribWeight;
                out_color[3 * pix_id + 1] += gd.rgb.y() * contribWeight;
                out_color[3 * pix_id + 2] += gd.rgb.z() * contribWeight;

                if (std::abs(gd.view_z) > 1e-6f) {
                    invdepth[pix_id] += (1.0f / gd.view_z) * contribWeight;
                }

                final_T[pix_id] = T * (1.0f - alpha);

                n_contrib[pix_id] += 1;
            }
        }
    }

    uploadAndDrawBuffer(W, H, out_color);
}
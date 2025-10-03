#pragma once

#include <Eigen/Dense>

#include <vector>
#include <GL/gl.h>

// sh constants
constexpr float SH_C0      = 0.28209479177387814f;
constexpr float SH_C1      = 0.4886025119029199f;
constexpr float SH_C2[5]   = {
    1.0925484305920792f,
    -1.0925484305920792f,
    0.31539156525252005f,
    -1.0925484305920792f,
    0.5462742152960396f
};
constexpr float SH_C3[7]   = {
    -0.5900435899266435f,
    2.890611442640554f,
    -0.4570457994644658f,
    0.3731763325901154f,
    -0.4570457994644658f,
    1.445305721320277f,
    -0.5900435899266435f
};

// eval spherical harmonics color
inline Eigen::Vector3f evalSH(
    int degree,
    const Eigen::Vector3f& dir,
    const Eigen::Vector3f* coeffs)
{
    Eigen::Vector3f result = SH_C0 * coeffs[0];

    if (degree > 0) {
        float x = dir.x();
        float y = dir.y();
        float z = dir.z();

        result += -SH_C1 * y * coeffs[1]
                  + SH_C1 * z * coeffs[2]
                  - SH_C1 * x * coeffs[3];

        if (degree > 1) {
            float xx = x*x, yy = y*y, zz = z*z;
            float xy = x*y, yz = y*z, xz = x*z;

            result += SH_C2[0] * xy * coeffs[4]
                      + SH_C2[1] * yz * coeffs[5]
                      + SH_C2[2] * (2.0f*zz - xx - yy) * coeffs[6]
                      + SH_C2[3] * xz * coeffs[7]
                      + SH_C2[4] * (xx - yy) * coeffs[8];

            if (degree > 2) {
                result += SH_C3[0] * y * (3.0f*xx - yy) * coeffs[9]
                          + SH_C3[1] * xy * z * coeffs[10]
                          + SH_C3[2] * y * (4.0f*zz - xx - yy) * coeffs[11]
                          + SH_C3[3] * z * (2.0f*zz - 3.0f*(xx+yy)) * coeffs[12]
                          + SH_C3[4] * x * (4.0f*zz - xx - yy) * coeffs[13]
                          + SH_C3[5] * z * (xx - yy) * coeffs[14]
                          + SH_C3[6] * x * (xx - 3.0f*yy) * coeffs[15];
            }
        }
    }

    result.array() += 0.5f;

    return result.cwiseMax(0.0f);
}

inline void uploadAndDrawBuffer(GLuint tex, int W, int H, const std::vector<float>& buffer)
{
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, W, H, 0, GL_RGB, GL_FLOAT, buffer.data());

    // Save previous matrices
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex);

    glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2f(-1, -1);
        glTexCoord2f(1, 0); glVertex2f( 1, -1);
        glTexCoord2f(1, 1); glVertex2f( 1,  1);
        glTexCoord2f(0, 1); glVertex2f(-1,  1);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);

    // Restore matrices
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

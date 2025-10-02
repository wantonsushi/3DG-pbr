#include "rasterizer.h"
#include <GL/gl.h>
#include <cmath>

// simple UV sphere drawer (no GLU)
void draw_sphere(const Eigen::Vector3f& pos, float radius, const Eigen::Vector3f& color, int segments = 16) {
    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());
    glColor3f(color.x(), color.y(), color.z());

    for (int i = 0; i <= segments; ++i) {
        float lat0 = M_PI * (-0.5f + float(i - 1) / segments);
        float z0 = sin(lat0);
        float zr0 = cos(lat0);

        float lat1 = M_PI * (-0.5f + float(i) / segments);
        float z1 = sin(lat1);
        float zr1 = cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= segments; ++j) {
            float lng = 2 * M_PI * float(j - 1) / segments;
            float x = cos(lng);
            float y = sin(lng);

            glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0);
            glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1);
        }
        glEnd();
    }

    glPopMatrix();
}

void Rasterizer::draw_spheres() {
    for (const auto& s : spheres) {
        draw_sphere(s.position, s.radius, s.color);
    }
}

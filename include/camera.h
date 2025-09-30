#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "ray.h"

class Camera {
public:
    Eigen::Vector3f position;
    Eigen::Vector3f view_dir;
    Eigen::Vector3f up;
    Eigen::Vector3f right;

    float fov_y = 45.0f;
    float aspect = 16.0f/9.0f;
    float near_plane = 0.1f;
    float far_plane = 100.0f;

    Camera() = default;

    void set_perspective(float fovY, float aspectRatio, float nearP, float farP) {
        fov_y = fovY;
        aspect = aspectRatio;
        near_plane = nearP;
        far_plane = farP;
    }

    void look_at(const Eigen::Vector3f& pos, const Eigen::Vector3f& target, const Eigen::Vector3f& world_up) {
        position = pos;
        view_dir = (target - pos).normalized();
        right = view_dir.cross(world_up).normalized();
        up = right.cross(view_dir).normalized();
    }

    void rotate(float yaw, float pitch) {
        // rotate around world up 
        Eigen::AngleAxisf yawRot(yaw, Eigen::Vector3f::UnitY());
        // rotate around camera right
        Eigen::AngleAxisf pitchRot(pitch, right);

        view_dir = (yawRot * pitchRot * view_dir).normalized();
        right = view_dir.cross(Eigen::Vector3f::UnitY()).normalized();
        up = right.cross(view_dir).normalized();
    }

    // view matrix for rasterization
    Eigen::Matrix4f view_matrix() const {
        Eigen::Matrix3f R;
        R.col(0) = right;
        R.col(1) = up;
        R.col(2) = -view_dir;
        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
        view.block<3,3>(0,0) = R.transpose();
        view.block<3,1>(0,3) = -R.transpose() * position;
        return view;
    }

    // projection matrix for rasterization
    Eigen::Matrix4f projection_matrix() const {
        float tanHalfFovy = std::tan(fov_y * M_PI / 360.0f);
        Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
        proj(0,0) = 1.0f / (aspect * tanHalfFovy);
        proj(1,1) = 1.0f / tanHalfFovy;
        proj(2,2) = -(far_plane + near_plane) / (far_plane - near_plane);
        proj(2,3) = -(2.0f * far_plane * near_plane) / (far_plane - near_plane);
        proj(3,2) = -1.0f;
        return proj;
    }

    // pinhole ray (for ray tracer)
    Ray generate_ray(float px, float py) const {
        // px, py in [0,1] (normalized image coordinates)
        float ndc_x = 2.0f * px - 1.0f; // [-1,1]
        float ndc_y = 1.0f - 2.0f * py; // [-1,1], flip Y

        float tanHalfFovy = std::tan(fov_y * M_PI / 360.0f);
        Eigen::Vector3f ray_dir = (view_dir
                                  + ndc_x * aspect * tanHalfFovy * right
                                  + ndc_y * tanHalfFovy * up).normalized();
        return {position, ray_dir};
    }
};

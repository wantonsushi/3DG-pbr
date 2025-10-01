#pragma once

#include "splat_scene.h"
#include <fstream>
#include <string>
#include <stdexcept>

namespace splat {

struct PlyLoader {
    static SplatScene load(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file) throw std::runtime_error("Could not open " + filename);

        // --- Parse header ---
        std::string line;
        size_t vertex_count = 0;
        while (std::getline(file, line)) {
            if (line.rfind("element vertex", 0) == 0) {
                vertex_count = std::stoul(line.substr(15));
            }
            if (line == "end_header") break;
        }

        if (vertex_count == 0) throw std::runtime_error("PLY has no vertices");

        SplatScene scene;
        scene.positions.resize(vertex_count);
        scene.scales.resize(vertex_count);
        scene.rotations.resize(vertex_count);
        scene.opacity.resize(vertex_count);
        scene.sh_coeffs.resize(vertex_count);

        // --- Read binary records ---
        for (size_t i = 0; i < vertex_count; i++) {
            float buf[3 + 3 + 48 + 1 + 3 + 3]; 
            // pos + normals + SH(48) + opacity + scale(3) + rotation Euler(3)
            file.read(reinterpret_cast<char*>(buf), sizeof(buf));

            // positions
            scene.positions[i] = Eigen::Vector3f(buf[0], buf[1], buf[2]);

            // skip normals (buf[3..5])

            // SH coefficients (16 RGB triplets)
            int idx = 6;
            for (int sh = 0; sh < 16; sh++) {
                scene.sh_coeffs[i][sh] = Eigen::Vector3f(
                    buf[idx], buf[idx + 1], buf[idx + 2]
                );
                idx += 3;
            }

            // opacity
            scene.opacity[i] = buf[idx++];

            // scales
            scene.scales[i] = Eigen::Vector3f(buf[idx], buf[idx + 1], buf[idx + 2]);
            idx += 3;

            // rotation: Euler angles → quaternion
            float rx = buf[idx++];
            float ry = buf[idx++];
            float rz = buf[idx++];
            scene.rotations[i] = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX())
                               * Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY())
                               * Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());
        }

        return scene;
    }
};

} // namespace splat

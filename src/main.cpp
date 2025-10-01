#include "viewer.h"
#include "splatloader.h"

#include <argparse/argparse.hpp>

#include <memory>
#include <random>

using namespace std;

int main(int argc, char** argv) {
    argparse::ArgumentParser parser("renderer_viewer");

    parser.add_argument("-i", "--input")
        .required()
        .help("Input .ply scene file");

    parser.add_argument("-p", "--print-timings")
        .default_value(true)
        .implicit_value(true)
        .help("Enable printing of per-frame timings (default: true)");

    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << parser;
        return -1;
    }

    std::string ply_file = parser.get<std::string>("--input");
    bool print_timings = parser.get<bool>("--print-frame-timings");

    // =================================================================================

    /*
    // Load scene for rasterizing
    shared_ptr<SplatScene> scene;
    try {
        scene = make_shared<SplatScene>(splat::PlyLoader::load(ply_file));
        std::cout << "Loaded scene with " << scene->size() << " splats.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error loading scene: " << e.what() << std::endl;
        return -1;
    }*/

    auto scene = make_shared<SplatScene>();
    scene->reserve(10);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    std::uniform_real_distribution<float> scale_dist(0.05f, 0.2f);
    std::uniform_real_distribution<float> opacity_dist(0.2f, 1.0f);

    for (int i = 0; i < 10; i++) {
        Eigen::Vector3f pos(dist(rng) * 2.0f, dist(rng) * 2.0f, -2.0f + dist(rng));
        Eigen::Vector3f sc(scale_dist(rng), scale_dist(rng), scale_dist(rng));
        Eigen::Quaternionf rot = Eigen::Quaternionf::UnitRandom();
        float op = opacity_dist(rng);

        std::array<Eigen::Vector3f, 16> sh{};
        for (auto &c : sh) {
            c = Eigen::Vector3f(dist(rng) * 0.5f + 0.5f, 
                                dist(rng) * 0.5f + 0.5f, 
                                dist(rng) * 0.5f + 0.5f); // RGB in [0,1]
        }

        scene->positions.push_back(pos);
        scene->scales.push_back(sc);
        scene->rotations.push_back(rot);
        scene->opacity.push_back(op);
        scene->sh_coeffs.push_back(sh);

        scene->covariances.push_back(Eigen::Matrix3f::Identity() * sc.mean());
        scene->bounding_radii.push_back(sc.norm());
    }

    std::cout << "Created fake scene with " << scene->size() << " splats.\n";
    // =================================================================================

    Viewer viewer(1280, 720, "Renderer Viewer", print_timings);
    viewer.set_scene(scene);

    if (!viewer.init()) {
        return -1;
    }

    viewer.run();

    viewer.shutdown();
    return 0;
}
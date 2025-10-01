#include "viewer.h"
#include "splatloader.h"

#include <argparse/argparse.hpp>

using namespace std;

int main(int argc, char** argv) {
    argparse::ArgumentParser parser("renderer_viewer");

    parser.add_argument("-i", "--input")
        .required()
        .help("Input .ply scene file");

    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << parser;
        return -1;
    }

    std::string ply_file = parser.get<std::string>("--input");

    // =================================================================================

    // Load scene for rasterizing
    shared_ptr<SplatScene> scene;
    try {
        scene = make_shared<SplatScene>(splat::PlyLoader::load(ply_file));
        std::cout << "Loaded scene with " << scene->size() << " splats.\n";
    } catch (const std::exception& e) {
        std::cerr << "Error loading scene: " << e.what() << std::endl;
        return -1;
    }

    Viewer viewer(1280, 720, "Renderer Viewer");
    viewer.set_scene(scene);

    if (!viewer.init()) {
        return -1;
    }

    viewer.run();

    viewer.shutdown();
    return 0;
}
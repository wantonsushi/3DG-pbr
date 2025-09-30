#include "viewer.h"

using namespace std;

int main() {
    Viewer viewer(1280, 720, "Renderer Viewer");

    if (!viewer.init()) {
        return -1;
    }

    viewer.run();

    viewer.shutdown();
    return 0;
}
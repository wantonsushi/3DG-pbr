#pragma once

#include "camera.h"
#include "splat_scene.h"

#include <GLFW/glfw3.h>
#include <Eigen/Core>
#include <Eigen/Geometry> 

class Viewer {
public:
    Viewer(int width, int height, const std::string& title, bool print_timings = true);
    ~Viewer();

    bool init();
    void run();
    void shutdown();

    void set_scene(std::shared_ptr<SplatScene> scene_ptr) {
        scene = scene_ptr;
    }

private:
    GLFWwindow* window = nullptr;
    int width, height;
    std::string title;

    Camera camera;
    std::shared_ptr<SplatScene> scene; 

    bool rasterize = true;
    bool printFrameTimings = true;

    bool r_pressed_last_frame = false;

    // Input handling
    bool firstMouse = true;
    double lastX = 0.0, lastY = 0.0;
    float moveSpeed = 2.5f;
    float mouseSensitivity = 0.002f;

    void processInput(float deltaTime);
    static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
};


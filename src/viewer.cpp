#include "viewer.h"
#include <iostream>

Viewer::Viewer(int w, int h, const std::string& t)
    : width(w), height(h), title(t) {}

Viewer::~Viewer() {}

bool Viewer::init() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSetWindowUserPointer(window, this);
    glfwSetCursorPosCallback(window, Viewer::mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // initialize camera defaults
    camera.set_perspective(45.0f, float(width) / float(height), 0.1f, 100.0f);
    camera.look_at({0, 0, 3}, {0, 0, 0}, {0, 1, 0});

    return true;
}

void Viewer::run() {
    float lastFrame = 0.0f;

    while (!glfwWindowShouldClose(window)) {
        float currentFrame = float(glfwGetTime());
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(deltaTime);

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // call rasterize(camera) or raytrace(camera)

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void Viewer::shutdown() {
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Viewer::processInput(float deltaTime) {
    float velocity = moveSpeed * deltaTime;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.position += camera.view_dir * velocity;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.position -= camera.view_dir * velocity;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.position -= camera.right * velocity;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.position += camera.right * velocity;
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera.position += camera.up * velocity;
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
        camera.position -= camera.up * velocity;

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

void Viewer::mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    Viewer* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    Camera& cam = viewer->camera;

    if (viewer->firstMouse) {
        viewer->lastX = xpos;
        viewer->lastY = ypos;
        viewer->firstMouse = false;
    }

    float xoffset = float(xpos - viewer->lastX);
    float yoffset = float(viewer->lastY - ypos); // invert Y
    viewer->lastX = xpos;
    viewer->lastY = ypos;

    xoffset *= viewer->mouseSensitivity;
    yoffset *= viewer->mouseSensitivity;

    cam.rotate(xoffset, yoffset);
}

#include "viewer.h"
#include <iostream>
#include <chrono>

#include "rasterizer.h"

Viewer::Viewer(int w, int h, const std::string& t, bool print_timings)
    : width(w), height(h), title(t), printFrameTimings(print_timings) {}

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
    //glfwSwapInterval(0);

    int framebufferWidth, framebufferHeight;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    glViewport(0, 0, framebufferWidth, framebufferHeight);

    glfwSetWindowUserPointer(window, this);
    glfwSetCursorPosCallback(window, Viewer::mouse_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // initialize camera defaults
    camera.set_perspective(45.0f, float(width) / float(height), 0.1f, 100.0f);
    camera.look_at({0, 0, 3}, {0, 0, 0}, {0, 1, 0});

    glEnable(GL_DEPTH_TEST);

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow* window, int width, int height) {
        glViewport(0, 0, width, height);
    });

    return true;
}

void Viewer::run() {
    float lastFrame = 0.0f;

    std::cout << "Rendering..." << std::endl;

    static Rasterizer raster;
    raster.setScene(scene.get());

    while (!glfwWindowShouldClose(window)) {
        float currentFrame = float(glfwGetTime());
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        if (printFrameTimings) {
            std::cout << "Frame time: " << deltaTime * 1000.0f << " ms" << std::endl;
        }

        processInput(deltaTime);

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // make a frame using rasterize or raytrace
        if(rasterize) {
            glMatrixMode(GL_PROJECTION);
            glLoadMatrixf(camera.projection_matrix().data());
            glMatrixMode(GL_MODELVIEW);
            glLoadMatrixf(camera.view_matrix().data());

            // render scene of Gaussians
            raster.draw_splats();
        }
        else {
            // ...
        }

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
        camera.position -= camera.up * velocity;
    if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS)
        camera.position += camera.up * velocity;

    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
        
    bool r_pressed = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
    if (r_pressed && !r_pressed_last_frame) {
        rasterize = !rasterize;
        if (rasterize)
            std::cout << "Switched to Rasterization mode." << std::endl;
        else
            std::cout << "Switched to Ray Tracing mode." << std::endl;
    }
    r_pressed_last_frame = r_pressed;
        
    
}

void Viewer::mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    Viewer* viewer = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    Camera& cam = viewer->camera;

    if (viewer->firstMouse) {
        viewer->lastX = xpos;
        viewer->lastY = ypos;
        viewer->firstMouse = false;
    }

    float xoffset = float(viewer->lastX - xpos);  // flip left/right
    float yoffset = float(ypos - viewer->lastY); // keep Y inverted
    viewer->lastX = xpos;
    viewer->lastY = ypos;

    xoffset *= viewer->mouseSensitivity;
    yoffset *= viewer->mouseSensitivity;

    cam.rotate(xoffset, yoffset);
}

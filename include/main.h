#pragma once
#include <functional>
#include <string>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

struct AppState {
    bool& pause_sim;
    bool& save_frames;
    bool& save_single_frame;
    double& displayW;
    double& displayH;
    GLFWwindow* window;
};

void keyCallback(GLFWwindow* w, int key, int sc, int action, int mods);

GLuint compile_shader(GLenum type, const char* src);

GLuint create_program(const std::string& vs_path, const std::string& fs_path);

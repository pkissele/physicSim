#pragma once
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

struct VAOResult {
    GLuint vao, vbo;
};

GLFWwindow* initWindow(int width, int height, const char* title);

VAOResult createStaticVAO(const float* data, size_t size);

GLuint attachVBO(int attribIndex, int components, int count);

void uploadBuffer(GLuint buf, const void* data, size_t bytes);

void drawVAO(GLuint program, GLuint vao, GLenum mode, int count);

GLuint compile_shader(GLenum type, const char* src);

GLuint createProgram(const std::string& vs_path, const std::string& fs_path);

std::string keyName(int key);

std::string loadFile(const std::string& path);

void saveFramebufferPNG(const std::string& filename, int width, int height);

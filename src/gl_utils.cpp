#include "gl_utils.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <random>

#include <png.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>


using namespace std;



GLFWwindow* initWindow(int width, int height, const char* title) {
    if (!glfwInit()) { cerr << "Failed to init GLFW\n"; return nullptr; }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(width, height, title, NULL, NULL);
    if (!window) { cerr << "Failed to create window\n"; glfwTerminate(); return nullptr; }

    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {cerr << "Failed to initialize GLAD\n"; return nullptr; }

    glViewport(0, 0, width, height);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    return window;
}


VAOResult createStaticVAO(const float* data, size_t size) {
    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    return {vao, vbo};
}


GLuint attachVBO(int attribIndex, int components, int count) {
    GLuint vbo;
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * components * count, NULL, GL_STREAM_DRAW);
    glEnableVertexAttribArray((GLuint)attribIndex);
    glVertexAttribPointer((GLuint)attribIndex, components, GL_FLOAT, GL_FALSE, components * sizeof(float), (void*)0);
    return vbo;
}


void uploadBuffer(GLuint buf, const void* data, size_t bytes) {
    glBindBuffer(GL_ARRAY_BUFFER, buf);
    glBufferSubData(GL_ARRAY_BUFFER, 0, bytes, data);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void drawVAO(GLuint program, GLuint vao, GLenum mode, int count) {
    glUseProgram(program);
    glBindVertexArray(vao);
    glDrawArrays(mode, 0, count);
    glBindVertexArray(0);
    glUseProgram(0);
}


GLuint compile_shader(GLenum type, const char* src) {
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char buf[1024];
        glGetShaderInfoLog(s, 1024, nullptr, buf);
        cerr << "Shader compile error: " << buf << endl;
        exit(1);
    }
    return s;
}


GLuint createProgram(const string& vs_path, const string& fs_path) {
    string vs_src = loadFile(vs_path);
    string fs_src = loadFile(fs_path);

    GLuint vs = compile_shader(GL_VERTEX_SHADER, vs_src.c_str());
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fs_src.c_str());

    GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);

    GLint ok;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (!ok) {
        char buf[1024];
        glGetProgramInfoLog(program, 1024, nullptr, buf);
        cerr << "Program link error: " << buf << endl;
        exit(1);
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
    return program;
}


string keyName(int key) {
    const char* name = glfwGetKeyName(key, 0);
    if (name) return string(name);
    switch (key) {
        case GLFW_KEY_ESCAPE: return "Escape";
        case GLFW_KEY_SPACE:  return "Space";
        case GLFW_KEY_UP:     return "Up";
        case GLFW_KEY_DOWN:   return "Down";
        case GLFW_KEY_LEFT:   return "Left";
        case GLFW_KEY_RIGHT:  return "Right";
        case GLFW_KEY_ENTER:  return "Enter";
        default: return "Unknown(" + to_string(key) + ")";
    }
}


string loadFile(const string& path) {
    ifstream in(path);
    if (!in.is_open()) {
        cerr << "Failed to open file: " << path << endl;
        exit(1);
    }
    stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}


void saveFramebufferPNG(const string& filename, int width, int height) {
    vector<unsigned char> raw(width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, raw.data());

    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        cerr << "Failed to open " << filename << " for writing\n";
        return;
    }

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        fclose(fp);
        cerr << "Failed to create PNG write struct\n";
        return;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        fclose(fp);
        cerr << "Failed to create PNG info struct\n";
        return;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        fclose(fp);
        cerr << "PNG write error\n";
        return;
    }

    png_init_io(png_ptr, fp);

    png_set_IHDR(
        png_ptr, info_ptr,
        width, height,
        8,
        PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_BASE,
        PNG_FILTER_TYPE_BASE
    );

    png_write_info(png_ptr, info_ptr);

    vector<png_bytep> row_pointers(height);
    for (int y = 0; y < height; y++) {
        int flipped = height - 1 - y;
        row_pointers[y] = (png_bytep)&raw[flipped * width * 3];
    }

    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);

    cout << "Saved PNG: " << filename << "\n";
}

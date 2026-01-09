#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <random>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <filesystem>

#include <thread>

#include <Eigen/Dense>

#include <png.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "simulation.h"
#include "utils.h"

using namespace std;
using Vec2 = Eigen::Vector2d;

// view
const double viewW = 20.0;
const double viewH = 20.0;

double displayW = viewW;
double displayH = viewH;

// window size
const int screenW = 900;
const int screenH = 900;

// helper: compile shader and link program
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

GLuint create_program(const string& vs_path, const string& fs_path) {
    string vs_src = load_file(vs_path);
    string fs_src = load_file(fs_path);

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

int main(int argc, char** argv) {
    // Simulation parameters
    const int N = 5000;
    const double mass = 0.0005;
    const double size = 6;

    // runtime options
    bool save_frames = false;
    bool pause_sim = false;
    bool save_single_frame = false;
    int save_counter = 0;

    // init glfw
    if (!glfwInit()) {
        cerr << "Failed to init GLFW" << endl;
        return -1;
    }

    // Request OpenGL 3.3 core
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(screenW, screenH, "N-body Live Viewer", NULL, NULL);
    if (!window) {
        cerr << "Failed to create window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Load GL functions using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        cerr << "Failed to initialize GLAD\n";
        return -1;
    }

    glViewport(0,0,screenW,screenH);

    float quadVerts[] = {
    -1.0f, -1.0f,
        1.0f, -1.0f,
    -1.0f,  1.0f,
        1.0f,  1.0f
    };

    GLuint quadVAO, quadVBO;
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);


    GLuint program = create_program("shaders/vertex.glsl", "shaders/fragment.glsl");
    GLuint bgProgram = create_program("shaders/bg_vertex.glsl", "shaders/bg_fragment.glsl");

    GLuint vao, vbo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    // dynamic buffer large enough for N points (2 floats each)
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * N, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    GLuint velVBO;
    glGenBuffers(1, &velVBO);
    glBindBuffer(GL_ARRAY_BUFFER, velVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * N, NULL, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    GLuint sizeVBO;
    glGenBuffers(1, &sizeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, sizeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * N, NULL, GL_DYNAMIC_DRAW);

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(float), (void*)0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // input state via lambda callbacks
    glfwSetWindowUserPointer(window, &save_frames);
    glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int sc, int action, int mods){
        if (action == GLFW_PRESS) {
            bool* save_ptr = (bool*)glfwGetWindowUserPointer(w);
            if (key == GLFW_KEY_ESCAPE) {
                glfwSetWindowShouldClose(w, GLFW_TRUE);
            } else if (key == GLFW_KEY_S) {
                *save_ptr = !(*save_ptr);
                cout << "Toggle continuous saving: " << (*save_ptr ? "ON" : "OFF") << endl;
            } else if (key == GLFW_KEY_SPACE) {
            } else if (key == GLFW_KEY_P) {
            }
        }
    });


    // Make output folder if saving
    string outdir = "outputFrames";
    if (!filesystem::exists(outdir)) {
        filesystem::create_directory(outdir);
    }
    const int FPS_TARGET = 60;

    double t = 0.0;
    // double dt = 1e-2; 
    double dt = (double)1/FPS_TARGET;
    const double FRAME_TARGET = 1000/FPS_TARGET;

    NBodySimulation sim(N, mass, size, viewW, viewH);

    // parse optional args
    for (int i=1;i<argc;i++){
        string arg(argv[i]);
        if (arg == "--save") save_frames = true;
        if (arg.rfind("--dt=",0) == 0) {
            dt = stod(arg.substr(5));
            // timeSteps = (int)round((endT - t) / dt);
            cout << "Using dt = " << dt << endl;
            // cout << "Using dt = " << dt << ", timeSteps=" << timeSteps << endl;
        }
    }

    // render loop variables
    double sim_time = 0.0;
    int frame_idx = 0;
    const float point_screen_size = 6.0f;

    // main loop
    double lastTime = glfwGetTime();
    int step = 0;

    bool INFO_FLAG = false;
    cout << "Starting main loop (press ESC to quit, S to toggle saving, SPACE to pause/resume, P to save single frame)" << endl;
    while (!glfwWindowShouldClose(window)) {
        // INFO_FLAG = (step%30 == 0);
        // Simulation stepping (we step a small number of physics steps per render to keep sim stable)
        if (!pause_sim) {
            sim.step(dt, INFO_FLAG);
        }

        const auto& bodies = sim.getBodies();

        int aliveN = sim.getAlive();

        cout << aliveN << endl;

        // Prepare data for GPU: convert body positions to NDC [-1,1]
        vector<float> ndc(2 * aliveN);
        for (int i = 0; i < aliveN; ++i) {
            float nx = (float)(((bodies[i].pos[0] + 0.5*(displayW-viewW)) / displayW) * 2.0 - 1.0);
            float ny = (float)(((bodies[i].pos[1] + 0.5*(displayH-viewH)) / displayH) * 2.0 - 1.0);
            ndc[2*i+0] = nx;
            ndc[2*i+1] = ny;
        }

        vector<float> vel(2 * aliveN);
        for(int i = 0; i < aliveN; ++i) {
            float vx = (float)(bodies[i].vel[0]);
            float vy = (float)(bodies[i].vel[1]);
            vel[2*i+0] = vx;
            vel[2*i+1] = vy;
        }

        vector<float> sizeNDC(aliveN);
        for(int i = 0; i < aliveN; ++i) {
            float s = (float)(bodies[i].size * (viewH/displayH));
            sizeNDC[i] = s;
            // cout << s << endl;
        }

        // upload positions to vbo
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 2 * aliveN, ndc.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // upload velocities to vbo
        glBindBuffer(GL_ARRAY_BUFFER, velVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * 2 * aliveN, vel.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        glBindBuffer(GL_ARRAY_BUFFER, sizeVBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float) * aliveN, sizeNDC.data());
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        // render
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(bgProgram);
        glBindVertexArray(quadVAO);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4); // or GL_TRIANGLES if using 2 triangles
        glBindVertexArray(0);

        glUseProgram(program);
        glBindVertexArray(vao);

        // set uniforms
        // GLint loc_size = glGetUniformLocation(program, "pointSize");
        // glUniform1f(loc_size, point_screen_size);

        GLint loc_color = glGetUniformLocation(program, "color");
        glUniform3f(loc_color, 1.0f, 1.0f, 1.0f);

        glDrawArrays(GL_POINTS, 0, aliveN);

        glBindVertexArray(0);
        glUseProgram(0);

        glfwSwapBuffers(window);

        // saving
        if (save_frames || save_single_frame) {
            ostringstream ss;
            ss << outdir << "/frame_" << setw(5) << setfill('0') << frame_idx << ".png";
            save_framebuffer_png(ss.str(), screenW, screenH);
            ++frame_idx;
            save_single_frame = false;
        }

        // key toggles
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            pause_sim = !pause_sim;
            this_thread::sleep_for(chrono::milliseconds(150));
            cout << "Pause: " << (pause_sim ? "ON" : "OFF") << endl;
        }
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
            save_frames = !save_frames;
            this_thread::sleep_for(chrono::milliseconds(150));
            cout << "Continuous save: " << (save_frames ? "ON" : "OFF") << endl;
        }
        if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
            save_single_frame = true;
            this_thread::sleep_for(chrono::milliseconds(100));
        }
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            cout << "up arrow" << endl;
            displayW *= 0.66;
            displayH *= 0.66;
        }
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            cout << "down arrow" << endl;
            displayW *= 1.5;
            displayH *= 1.5;
        }
 
        double now = glfwGetTime();
        double elapsed = (now - lastTime) * 1000.0;
        lastTime = now;

        if (elapsed < FRAME_TARGET) {
            this_thread::sleep_for(chrono::milliseconds((long long)(FRAME_TARGET - elapsed)));
        } else {
            cout << elapsed << " ms of frametime" << endl;
        }
        step += 1;
    }

    // cleanup
    glDeleteBuffers(1, &sizeVBO);
    glDeleteBuffers(1, &velVBO);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(program);

    glfwDestroyWindow(window);
    glfwTerminate();

    cout << "Program terminated.\n";
    return 0;
}

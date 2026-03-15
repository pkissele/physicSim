#include "main.h"

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>

#include <png.h>
#include <Eigen/Dense>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "tree.h"
#include "gl_utils.h"


using namespace std;
using Vec2 = Eigen::Vector2d;



// view
const double viewW = 20.0;
const double viewH = 20.0;

// Current display scaling
double displayW = viewW;
double displayH = viewH;

// Current display shift
double displayX = 0;
double displayY = 0;

// window parameters
const int screenW = 1500;
const int screenH = 1500;

// basic logging
const bool LOG_TIME = true;
const bool LOG_ENERGY = true;

// Simulation parameters
const int N = 200000;
const double mass = 0.00005;
const double bodySize = 5;
const int FPS_TARGET = 60;



int main(int argc, char** argv) {
    GLFWwindow* window = initWindow(screenW, screenH, "Galaxy Sim");
    if (!window) return -1;

    float quadVerts[] = {-1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f};
    auto [quadVAO, quadVBO] = createStaticVAO(quadVerts, sizeof(quadVerts));

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    GLuint posVBO  = attachVBO(0, 2, N);  // positions
    GLuint velVBO  = attachVBO(1, 2, N);  // velocities
    GLuint sizeVBO = attachVBO(2, 1, N);  // sizes

    // Unbind VAO (important to prevent overwriting) and VBO
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    GLuint program = createProgram("shaders/vertex.glsl", "shaders/fragment.glsl");
    GLuint bgProgram = createProgram("shaders/bg_vertex.glsl", "shaders/bg_fragment.glsl");


    // Runtime options
    bool save_frames = false;
    bool pause_sim = false;
    bool save_single_frame = false;
    int save_counter = 0;

    // parse optional args
    for (int i=1;i<argc;i++){
        string arg(argv[i]);
        if (arg == "--save") save_frames = true;
    }
    string outdir = "outputFrames";
    if (!filesystem::exists(outdir)) filesystem::create_directory(outdir);

    // define keybinds
    AppState state{pause_sim, save_frames, save_single_frame, displayW, displayH, window};
    glfwSetWindowUserPointer(window, &state);
    glfwSetKeyCallback(window, keyCallback);


    // Runtime variables
    double t = 0.0;
    double dt = (double)1/FPS_TARGET;
    const double FRAME_TARGET = 1000.0/FPS_TARGET;
    double sim_time = 0.0;
    int frame_idx = 0;
    double lastTime = glfwGetTime();

    // Initialize simulation
    quadTreeSim sim(N, mass, bodySize, viewW, viewH);

    int step = 0;
    bool INFO_FLAG = false;

    cout << "Starting main loop (press ESC to quit, S to toggle saving, SPACE to pause/resume, P to save single frame)" << endl;
    while (!glfwWindowShouldClose(window)) {
        // stagger logging
        if (LOG_ENERGY) INFO_FLAG = (step%30 == 0);

        // simulation step
        if (!pause_sim) sim.step(dt, INFO_FLAG);

        // pull info
        const auto& bodies = sim.getBodies();
        int aliveN = sim.getAlive();

        // convert body positions to NDC [-1,1] for GPU
        vector<float> posNDC(2 * aliveN);
        vector<float> velNDC(2 * aliveN);
        vector<float> sizeNDC(aliveN);

        for (int i = 0; i < aliveN; ++i) {
            posNDC[2*i+0] = (float)(((bodies[i].pos[0] + 0.5*(displayW-viewW)) / displayW) * 2.0 - 1.0);
            posNDC[2*i+1] = (float)(((bodies[i].pos[1] + 0.5*(displayH-viewH)) / displayH) * 2.0 - 1.0);
        }
        for (int i = 0; i < aliveN; ++i) {
            velNDC[2*i+0] = (float)(bodies[i].vel[0]);
            velNDC[2*i+1] = (float)(bodies[i].vel[1]);
        }
        for (int i = 0; i < aliveN; ++i) {
            sizeNDC[i] = (float)(bodies[i].size * (viewH/displayH));
        }


        // upload buffers
        uploadBuffer(sizeVBO, sizeNDC.data(), sizeof(float) * 1 * aliveN);
        uploadBuffer(velVBO, velNDC.data(), sizeof(float) * 2 * aliveN);
        uploadBuffer(posVBO, posNDC.data(), sizeof(float) * 2 * aliveN);

        // Draw background
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        drawVAO(bgProgram, quadVAO, GL_TRIANGLE_STRIP, 4);

        // Draw bodies
        glUseProgram(program);
        GLint loc = glGetUniformLocation(program, "color");
        glUniform3f(loc, 1.0f, 1.0f, 1.0f);
        drawVAO(program, vao, GL_POINTS, aliveN);

        // Put to screen
        glfwSwapBuffers(window);

        // keybinds
        glfwPollEvents();

        // saving
        if (save_frames || save_single_frame) {
            ostringstream ss;
            ss << outdir << "/frame_" << setw(5) << setfill('0') << frame_idx << ".png";
            saveFramebufferPNG(ss.str(), screenW, screenH);
            ++frame_idx;
            save_single_frame = false;
        }

        // frame timing
        double now = glfwGetTime();
        double elapsed = (now - lastTime) * 1000.0;
        lastTime = now;

        if (LOG_TIME) cout << fixed << elapsed << " ms of frametime" << endl;
        if (elapsed < FRAME_TARGET) this_thread::sleep_for(chrono::milliseconds((long long)(FRAME_TARGET - elapsed)));

        step += 1;
    }

    glDeleteBuffers(1, &sizeVBO);
    glDeleteBuffers(1, &velVBO);
    glDeleteBuffers(1, &posVBO);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(program);
    glfwDestroyWindow(window);
    glfwTerminate();
    cout << "Program terminated.\n";
    return 0;
}


void keyCallback(GLFWwindow* w, int key, int sc, int action, int mods) {
    if (action != GLFW_PRESS) return;
    auto* s = (AppState*)glfwGetWindowUserPointer(w);
    switch (key) {
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(w, GLFW_TRUE); break;
        case GLFW_KEY_SPACE:  s->pause_sim = !s->pause_sim; break;
        case GLFW_KEY_J:      s->save_frames = !s->save_frames; break;
        case GLFW_KEY_P:      s->save_single_frame = true; break;
        case GLFW_KEY_UP:     s->displayW *= 0.66; s->displayH *= 0.66; break;
        case GLFW_KEY_DOWN:   s->displayW *= 1.5;  s->displayH *= 1.5;  break;
    }
    cout << keyName(key) << " pressed" << endl;
}

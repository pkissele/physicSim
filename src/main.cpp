#include "main.h"

#include <chrono>
#include <iostream>
#include <ratio>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <thread>
#include <atomic>

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
const bool LOG_GUI_TIME = false;
const bool LOG_SIM_TIME = true;
const bool LOG_ENERGY = false;

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
    bool save_single_frame = false;

    // Simulation/window state
    globalState state{save_frames, save_single_frame, displayW, displayH, window};


    // parse optional args
    for (int i=1;i<argc;i++) if ((string)argv[i] == "--save") save_frames = true;

    string outdir = "outputFrames";
    if (!filesystem::exists(outdir)) filesystem::create_directory(outdir);

    // define keybinds
    glfwSetWindowUserPointer(window, &state);
    glfwSetKeyCallback(window, keyCallback);


    // Runtime variables
    double dt = (double)1/FPS_TARGET;
    const double FRAME_TARGET = 1000.0/FPS_TARGET;
    int frame_idx = 0;
    double lastTime = glfwGetTime();
    int lastSavedFrame = -1;

    // Initialize simulation
    quadTreeSim sim(N, mass, bodySize, viewW, viewH);


    // Threading setup
    state.buffers[0].resize(N);
    state.buffers[1].resize(N);
    const auto& initBodies = sim.getBodies();
    copy(initBodies.begin(), initBodies.end(), state.buffers[0].begin());
    atomic<bool> running{true};


    // Start simulation thread
    thread simThread(simulate, ref(sim), ref(state), ref(running), dt, LOG_ENERGY, LOG_SIM_TIME);


    // instantiate NDC
    vector<float> posNDC(2 * N);
    vector<float> velNDC(2 * N);
    vector<float> sizeNDC(N);

    // get uniform
    GLint colorLoc = glGetUniformLocation(program, "color");
    int aliveN = N;
    const vector<Body>* bodies = &state.buffers[0];

    cout << "Starting main loop (press ESC to quit, S to toggle saving, SPACE to pause/resume, P to save single frame)" << endl;
    while (!glfwWindowShouldClose(window)) {
        // reuse data if nothing new
        if (state.newFrame.exchange(false)) {
            int ind;
            {
                lock_guard<mutex> lock(state.swapMutex);
                ind = state.readInd.load();
            }
            bodies = &state.buffers[ind];
            aliveN = bodies->size();

            posNDC .resize(2 * aliveN);
            velNDC .resize(2 * aliveN);
            sizeNDC.resize(1 * aliveN);
        }

        // convert body positions to NDC [-1,1] for GPU
        const auto& b = *bodies;
        for (int i = 0; i < aliveN; ++i) {
            posNDC[2*i+0] = (float)(((b[i].pos[0] + 0.5*(displayW-viewW)) / displayW) * 2.0 - 1.0);
            posNDC[2*i+1] = (float)(((b[i].pos[1] + 0.5*(displayH-viewH)) / displayH) * 2.0 - 1.0);
            velNDC[2*i+0] = (float)b[i].vel[0];
            velNDC[2*i+1] = (float)b[i].vel[1];
            sizeNDC[i]    = (float)(b[i].size * (viewH / displayH));
        }

        uploadBuffer(posVBO,  posNDC.data(),  sizeof(float) * 2 * aliveN);
        uploadBuffer(velVBO,  velNDC.data(),  sizeof(float) * 2 * aliveN);
        uploadBuffer(sizeVBO, sizeNDC.data(), sizeof(float) * 1 * aliveN);

        // Draw background
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        drawVAO(bgProgram, quadVAO, GL_TRIANGLE_STRIP, 4);

        // Draw bodies
        glUseProgram(program);
        glUniform3f(colorLoc, 1.0f, 1.0f, 1.0f);
        drawVAO(program, vao, GL_POINTS, aliveN);

        // Put to screen
        glfwSwapBuffers(window);
        // keybinds
        glfwPollEvents();

        // saving
        if (save_frames || save_single_frame) {
            int curFrame = state.simStep.load();
            if(curFrame != lastSavedFrame) {
                ostringstream ss;
                ss << outdir << "/frame_" << setw(5) << setfill('0') << frame_idx << ".png";
                saveFramebufferPNG(ss.str(), screenW, screenH);
                ++frame_idx;
                save_single_frame = false;
            }
        }

        // frame timing
        double now = glfwGetTime();
        double elapsed = (now - lastTime) * 1000.0;
        lastTime = now;

        if (LOG_GUI_TIME) cout << fixed << setprecision(2) << elapsed << " ms of frametime" << endl;
        if (elapsed < FRAME_TARGET) this_thread::sleep_for(chrono::milliseconds((long long)(FRAME_TARGET - elapsed)));
    }

    running = false;
    simThread.join();

    glDeleteBuffers(1, &sizeVBO);
    glDeleteBuffers(1, &velVBO);
    glDeleteBuffers(1, &posVBO);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(program);
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}



void simulate(quadTreeSim& sim, globalState& shared, atomic<bool>& running, double dt, bool LOG_ENERGY, bool LOG_SIM_TIME) {
    int step = 0;
    bool INFO_FLAG = false;
    while (running) {
        if(shared.paused) {
            this_thread::sleep_for(chrono::milliseconds(10));
            continue;
        }
        if (LOG_ENERGY) INFO_FLAG = (step % 30 == 0);

        auto start = chrono::high_resolution_clock::now();

        sim.step(dt, INFO_FLAG);

        auto end = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double, milli>(end - start).count();
        if (LOG_SIM_TIME) cout << "simulation step took "<< fixed << setprecision(2) << elapsed << " ms" << endl;

        const auto& bodies = sim.getBodies();
        int aliveN = sim.getAlive();

        int writeInd = 1 - shared.readInd.load();
        shared.buffers[writeInd].resize(aliveN);
        copy(bodies.begin(), bodies.begin() + aliveN, shared.buffers[writeInd].begin());
        {
            lock_guard<mutex> lock(shared.swapMutex);
            shared.readInd.store(writeInd);
            shared.newFrame.store(true);
            shared.simStep++;
        }

        step++;
    }
}



void keyCallback(GLFWwindow* w, int key, int sc, int action, int mods) {
    if (action != GLFW_PRESS) return;
    auto* s = (globalState*)glfwGetWindowUserPointer(w);
    switch (key) {
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(w, GLFW_TRUE); break;
        case GLFW_KEY_Q:      glfwSetWindowShouldClose(w, GLFW_TRUE); break;
        case GLFW_KEY_SPACE:  s->paused = !s->paused.load(); break;
        case GLFW_KEY_J:      s->save_frames = !s->save_frames; break;
        case GLFW_KEY_P:      s->save_single_frame = true; break;
        case GLFW_KEY_UP:     s->displayW *= 0.66; s->displayH *= 0.66; break;
        case GLFW_KEY_DOWN:   s->displayW *= 1.5;  s->displayH *= 1.5;  break;
    }
    cout << keyName(key) << " pressed" << endl;
}

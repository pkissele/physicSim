#pragma once

namespace Consts {
    //
    // Functions
    //
    template<typename T> inline constexpr T sq(T x) { return x * x; }

    //
    // Physical constants
    //
    constexpr float G = 1;
    constexpr float kB = 1;
    constexpr float adiaGamma = 1.4;
    constexpr float PI = 3.1415926;


    //
    // Simulation constants
    //

    constexpr int N = 100000;
    constexpr double mass = 0.0005;
    constexpr double centMass = 20;
    constexpr double bodySize = 5;
    constexpr int runSteps = 1100;

    constexpr float theta = 0.7;
    constexpr float init_theta = 0.4; // more accurate initialization

    constexpr float gravEpsilon = 0.5;
    constexpr float gravEpsilonSq = gravEpsilon*gravEpsilon;

    constexpr float ESCAPE_CULL_RAD = 8.0f;

    constexpr int LEAF_CAPACITY = 32;
    constexpr int BUILD_TASK_SIZE = 4096;
    constexpr int ACCEL_TASK_SIZE = 2048;

    constexpr float MIN_NODE_SIZE = 1e-6f;

    constexpr float INIT_TEMP = 1;
    constexpr float NEIGHBOR_SKIN = 0.3f;
    constexpr float DENS_TO_PRESS = 0.006f;


    //
    // Engine constants
    //
    constexpr bool HEADLESS = false;
    constexpr int FPS_TARGET = 30;
    constexpr double FRAME_TARGET = 1000.0/FPS_TARGET;


    // view
    constexpr double viewW = 30.0;
    constexpr double viewH = 30.0;

    // window parameters
    constexpr int screenW = 1000;
    constexpr int screenH = 1000;

    // basic logging
    constexpr bool LOG_GUI_TIME = false; 
    constexpr int LOG_GUI_TIME_INTERVAL = 30;
    constexpr bool LOG_SIM_TIME = false;
    constexpr int LOG_SIM_TIME_INTERVAL = 5;
    constexpr bool LOG_ENERGY = false; 
    constexpr int LOG_ENERGY_INTERVAL = 1;

    constexpr int MASS_DISPLAY_THRESHOLD = 5;
    constexpr float VISIBILITY = 0.1;
}

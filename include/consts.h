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


    //
    // Simulation constants
    //
    constexpr float theta = 0.7;
    constexpr float init_theta = 0.4; // more accurate initialization

    constexpr float gravEpsilon = 0.05;
    constexpr float gravEpsilonSq = gravEpsilon*gravEpsilon;

    constexpr float ESCAPE_CULL_RAD = 8.0f;

    constexpr int LEAF_CAPACITY = 32;
    constexpr int BUILD_TASK_SIZE = 4096;
    constexpr int ACCEL_TASK_SIZE = 2048;

    const float MIN_NODE_SIZE = 1e-6f;
}

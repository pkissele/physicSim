#pragma once

#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

struct Node {
    float mass = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float comx = 0.0f;
    float comy = 0.0f;
    float halfSize = 0.0f;

    int firstChild = -1;
    int next = -1;
    int bIndex = -1;
};


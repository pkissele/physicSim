#pragma once

#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

struct Node {
    double mass = 0.0;
    Vec2 com = Vec2::Zero();
    Vec2 center = Vec2::Zero();
    double halfSize = 0.0;
    int children[4];

    int bIndex = -1;

    // Constructor
    Node() : children{-1, -1, -1, -1} {}
};


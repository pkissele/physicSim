#pragma once

#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

struct Node {
    double mass = 0.0;
    Vec2 com = Vec2::Zero();
    Vec2 center = Vec2::Zero();
    double halfSize = 0.0;
    int firstChild = -1;
    int next = -1;

    int bIndex = -1;

};


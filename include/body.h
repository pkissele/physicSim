#pragma once

#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

struct Body {
    Vec2 pos;
    Vec2 vel;
    float mass;
    float size;
};

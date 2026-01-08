#pragma once

#include <Eigen/Dense>

using Vec2 = Eigen::Vector2d;

struct Body {
    double mass;
    Vec2 pos;
    Vec2 vel;
    double size;
};

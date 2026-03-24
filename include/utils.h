#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>

#include "bodies.h"

using Vec2 = Eigen::Vector2d;

auto morton(Vec2 pos, Vec2 min, Vec2 max) -> uint32_t;

void randDisk(Bodies& b, double maxRad);

void randVels(Bodies& b, double maxRad);

void setOrbitalVel(Bodies& b, float cx, float cy);

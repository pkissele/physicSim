#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>

#include "bodies.h"

using Vec2 = Eigen::Vector2d;

uint32_t expandBits(uint32_t x);

uint32_t mortonCode(float x, float y, float minX, float minY, float invScale);

void randDisk(Bodies& b, double maxRad, double minRad);

void randVels(Bodies& b, double maxRad);

void setOrbitalVel(Bodies& b, float cx, float cy);

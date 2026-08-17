#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>

#include "bodies.h"

using Vec2 = Eigen::Vector2d;

void setRng();

void randDisk(Bodies& b, double maxRad, double minRad);

void randVels(Bodies& b, double maxRad);

void setOrbitalVel(Bodies& b, float cx, float cy);

#pragma once
#include <string>
#include <Eigen/Dense>

#include "body.h"

using Vec2 = Eigen::Vector2d;

auto morton(Vec2 pos, Vec2 min, Vec2 max) -> uint32_t;

void randDisk(Body* b, double maxRad, double centMass, double pMass, int N);

void randVels(Body* b, double maxRad);

void setOrbitalVel(Body* b, Vec2* acc, Vec2* centPos);

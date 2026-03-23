#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>

#include "body.h"
#include "bodies.h"

using Vec2 = Eigen::Vector2d;

auto morton(Vec2 pos, Vec2 min, Vec2 max) -> uint32_t;

void randDisk(Bodies* b, double maxRad);
void randDisk2(Body* b, double maxRad, double centMass, double pMass, int N);

void randVels(Bodies* b, double maxRad);
void randVels2(Body* b, double maxRad);

void setOrbitalVel(Bodies* b, std::vector<Vec2>& acc, Vec2 centPos);
void setOrbitalVel2(Body* b, Vec2* acc, Vec2* centPos);

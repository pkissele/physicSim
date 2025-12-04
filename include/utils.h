#pragma once
#include <string>
#include <Eigen/Dense>

#include "body.h"

using Vec2 = Eigen::Vector2d;

void randDisk(Body* b, double maxRad, double centMass);

std::string load_file(const std::string& path);

void save_framebuffer_png(const std::string& filename, int width, int height);

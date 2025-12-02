#pragma once
#include <string>

std::string load_file(const std::string& path);

void save_framebuffer_png(const std::string& filename, int width, int height);
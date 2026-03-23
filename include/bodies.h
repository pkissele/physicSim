#pragma once

#include <vector>

struct Bodies {
    int N;
    std::vector<float> px;
    std::vector<float> py;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> mass;
    std::vector<float> size;

    Bodies(int nVal) : N(nVal){
        px.resize(N, 0.0f);
        py.resize(N, 0.0f);
        vx.resize(N, 0.0f);
        vy.resize(N, 0.0f);
        mass.resize(N, 0.0f);
        size.resize(N, 0.0f);
    }
};

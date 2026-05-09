#pragma once

#include <cstdint>
#include <vector>

struct Bodies { int N; std::vector<float> mass;
    std::vector<float> size;

    std::vector<float> px;
    std::vector<float> py;
    std::vector<float> vx;
    std::vector<float> vy;
    std::vector<float> ax;
    std::vector<float> ay;
    std::vector<float> axNew;
    std::vector<float> ayNew;


    std::vector<uint8_t> diffuse; 
    std::vector<float> press;
    std::vector<float> dens;
    std::vector<float> densNew;
    std::vector<float> energy;
    std::vector<float> h;


    Bodies(int nVal) : N(nVal){
        mass.resize(N, 0.0f);
        size.resize(N, 0.0f);

        px.resize(N, 0.0f);
        py.resize(N, 0.0f);
        vx.resize(N, 0.0f);
        vy.resize(N, 0.0f);
        ax.resize(N, 0.0f);
        ay.resize(N, 0.0f);
        axNew.resize(N, 0.0f);
        ayNew.resize(N, 0.0f);

        diffuse.resize(N, 0);
        press.resize(N, 0.0f);
        dens.resize(N, 0.0f);
        densNew.resize(N, 0.0f);
        energy.resize(N, 0.0f);
        h.resize(N, 0.0f);
    }
    //
    // std::vector<std::vector<float>*> arrays() {
    //     return {&mass, &size, &px, &py, &vx, &vy, &ax, &ay, &axNew, &ayNew, &diffuse, &press, &dens, &densNew, &energy};
    // }
    auto arrays() {
        return std::make_tuple(
            &mass, &size, &px, &py, &vx, &vy, &ax, &ay, &axNew, &ayNew,
            &diffuse, &press, &dens, &densNew, &energy
        );
    }
};

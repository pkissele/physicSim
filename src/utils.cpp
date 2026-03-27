#include "utils.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <random>

#include <GLFW/glfw3.h>

#include "bodies.h"

// using namespace Consts;

using namespace std;

using Vec2 = Eigen::Vector2d;


mt19937_64 rngBody(random_device{}());
uniform_real_distribution<double> dist01(0.0, 1.0);


uint32_t expandBits(uint32_t x) {
    // expand 16 bit to 32 bit by interleaving zeros
    x &= 0x0000FFFF;
    x = (x | x << 8)  & 0x00FF00FF;
    x = (x | x << 4)  & 0x0F0F0F0F;
    x = (x | x << 2)  & 0x33333333;
    x = (x | x << 1)  & 0x55555555;
    return x;
}


// Optimal spacial ordering
uint32_t mortonCode(float x, float y, float minX, float minY, float invScale) {
    // Normalize to [0, 65535]
    uint32_t ix = (uint32_t)((x - minX) * invScale * 65535.0f);
    uint32_t iy = (uint32_t)((y - minY) * invScale * 65535.0f);
    ix = min(ix, 65535u);
    iy = min(iy, 65535u);
    return (expandBits(ix) | (expandBits(iy) << 1));
}


void randDisk(Bodies& b, double maxRad, double minRad) {
    for(int i = 0; i < b.N; ++i) {
        double theta = dist01(rngBody) * 2.0 * M_PI;
        // double r = (maxRad-minRad) * pow(dist01(rngBody), 1) + minRad;
        double r = (maxRad-minRad) * pow(dist01(rngBody), 0.5) + minRad;

        Vec2 pos(r * cos(theta), r * sin(theta));

        b.px[i] = pos[0];
        b.py[i] = pos[1];
    }
}


void randVels(Bodies& b, double maxRad) {
    for(int i = 0; i < b.N; i++) {
        Vec2 nVel(dist01(rngBody) * maxRad, dist01(rngBody) * maxRad);
        b.vx[i] = nVel[0];
        b.vy[i] = nVel[1];
    }
}


void setOrbitalVel(Bodies& b, float cx, float cy) {
    for(int i = 0; i < b.N; i++) {
        Vec2 pos(b.px[i],b.py[i]);
        Vec2 centPos(cx, cy);
        Vec2 r = pos - centPos;
        Vec2 acc(b.ax[i], b.ay[i]);
        double radAccel = acc.dot(r.normalized());

        Vec2 nVel(-r[1], r[0]);
        nVel = sqrt(abs(r.norm()*radAccel)) * nVel.normalized();
        b.vx[i] = nVel[0];
        b.vy[i] = nVel[1];
    }
}



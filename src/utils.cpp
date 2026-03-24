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


// Morton ordering for quadtree cache locality
auto morton(Vec2 pos, Vec2 min, Vec2 max) -> uint32_t {
    uint64_t x = (uint64_t)((pos[0] - min[0]) / (max[0] - min[0]) * 0xFFFFFFFF);
    uint64_t y = (uint64_t)((pos[1] - min[1]) / (max[1] - min[1]) * 0xFFFFFFFF);
    uint64_t code = 0;
    for (int i = 0; i < 32; i++) {
        code |= ((x >> i) & 1) << (2*i);
        code |= ((y >> i) & 1) << (2*i + 1);
    }
    return code;
};


void randDisk(Bodies& b, double maxRad) {
    for(int i = 0; i < b.N; ++i) {
        double theta = dist01(rngBody) * 2.0 * M_PI;
        double r = maxRad * pow(dist01(rngBody), 0.5);

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



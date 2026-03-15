#include "utils.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <random>

#include <GLFW/glfw3.h>

#include "body.h"
// #include "consts.h"

// using namespace Consts;

using namespace std;

using Vec2 = Eigen::Vector2d;

mt19937_64 rngBody(random_device{}());
uniform_real_distribution<double> dist01(0.0, 1.0);

void randDisk(Body* b, double maxRad, double centMass, double pMass, int N) {
    double theta = dist01(rngBody) * 2.0 * M_PI;
    // double r = 3.0;
    double r = maxRad * pow(dist01(rngBody), 0.5);

    // double totMass = centMass + pMass*N*pow(r, 2)/(pow(maxRad, 2));
    Vec2 pos(r * cos(theta), r * sin(theta));

    b->pos = pos;
}

void randVels(Body* b, double maxRad) {
    Vec2 nVel(dist01(rngBody) * maxRad, dist01(rngBody) * maxRad);
    b->vel = nVel;
}

void setOrbitalVel(Body* b, Vec2* acc, Vec2* centPos) {
    Vec2 r = b->pos - *centPos;
    double radAccel = (*acc).dot(r.normalized());

    Vec2 nVel(-r[1], r[0]);
    nVel = sqrt(abs(r.norm()*radAccel)) * nVel.normalized();
    b->vel = nVel;
}

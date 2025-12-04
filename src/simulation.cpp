#include "simulation.h"

#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "utils.h"
#include "body.h"
#include "consts.h"

using namespace Consts;

using namespace std;

// const double G = 1.0; // gravitational constant for simulation
// const double pi = 3.14159265358979323846;
const double collisionRad = 0.1;

double gravEpsilon = 0.05;
double gravEpsilon2 = pow(gravEpsilon, 2);

double stepFrac = 10; // Janky sub-stepping

mt19937 rng(random_device{}());
uniform_real_distribution<double> randDist(0.0, 1.0);

using Vec2 = Eigen::Vector2d;

NBodySimulation::NBodySimulation(int N_, double mass, double viewW_, double viewH_)
    : N(N_), viewW(viewW_), viewH(viewH_) {
    bodies.resize(N);
    for(int i = 0; i < N; ++i) {
        bodies[i].mass = mass;
    }

    // Create massive body
    bodies[0].mass = 10;
    // bodies[1].mass = 1;

    Vec2 comPos = {0, 0};
    Vec2 comVel = {0, 0};
    double totMass = 0;

    double initVelScale = 1;
    double diskScale = 4;

    for (int i = 0; i < N; ++i) {
        if(i != 0) {
            randDisk(&bodies[i], diskScale, bodies[0].mass);
        }
        // bodies[i].pos *= min(viewW, viewH)/4;
        bodies[i].vel *= initVelScale;
        // bodies[i].pos = randDisk() * min(viewW, viewH)/2;
        // bodies[i].pos = Vec2(randDist(rng) * viewW, randDist(rng) * viewH);
        // bodies[i].vel = randDisk() * initVelScale;
        // bodies[i].vel = Vec2(randDist(rng) * initVelScale, randDist(rng) * initVelScale);
        // bodies[i].vel = Vec2(0.0, 0.0);
        comPos += bodies[i].pos * bodies[i].mass;
        comVel += bodies[i].vel * bodies[i].mass;
        totMass += bodies[i].mass;
    }

    comPos = comPos / totMass;
    comVel = comVel / totMass;

    cout << comPos << endl;
    Vec2 center = {viewW_/2, viewH_/2};
    // Center to avoid drift
    for (int i = 0; i < N; ++i) {
        bodies[i].pos += -comPos + center;
        bodies[i].vel += -comVel;
    }

    double meanSpacing = sqrt(viewW * viewH / N);
    double gravEpsilon = 0 * meanSpacing;
    double gravEpsilon1 = gravEpsilon * gravEpsilon;


}

vector<Vec2> NBodySimulation::computeAccelerations(bool DO_INFO = false, double &potEnergy = *(new double)) {
    vector<Vec2> accel(N, Vec2(0.0, 0.0));
    if (DO_INFO) potEnergy = 0;

    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == j) continue;

            Vec2 dir = bodies[j].pos - bodies[i].pos;
            double dx = dir[0], dy = dir[1];
            double distSq = dx*dx + dy*dy;
            double r = sqrt(distSq + gravEpsilon2);
            double invDist = 1.0 / r;

            double gravMag = G * bodies[j].mass / (distSq + gravEpsilon2);
            accel[i][0] += gravMag * dx * invDist;
            accel[i][1] += gravMag * dy * invDist;

            if (DO_INFO && i < j) {
                potEnergy += -1 * G * bodies[j].mass * bodies[i].mass / pow(distSq + gravEpsilon2, 0.5);
            }
        }
    }

    return accel;
}


void NBodySimulation::step(double dtIn, bool DO_INFO) {
    double dt = dtIn / stepFrac;
    if (accel.size() != N) {
        accel = computeAccelerations();
    }

    for (int ss = 0; ss < stepFrac; ss++) {
        bool INFO_FLAG = (ss == 0 && DO_INFO);
        double kinEnergy = 0, potEnergy = 0;

        // half-step velocity
        for (int i = 0; i < N; ++i) {
            bodies[i].vel += 0.5 * accel[i] * dt;
        }

        // full-step position
        for (int i = 0; i < N; ++i) {
            bodies[i].pos += bodies[i].vel * dt;
        }

        // compute new accelerations
        vector<Vec2> accelNew;
        if (INFO_FLAG) accelNew = computeAccelerations(true, potEnergy);
        else accelNew = computeAccelerations();

        // complete velocity update
        for (int i = 0; i < N; ++i) {
            bodies[i].vel += 0.5 * accelNew[i] * dt;

            if (INFO_FLAG) {
                kinEnergy += 0.5 * bodies[i].mass * (pow(bodies[i].vel[0], 2) + pow(bodies[i].vel[1], 2));
            }
        }

        if (INFO_FLAG) {
            cout << "Energy: " << scientific << kinEnergy + potEnergy << endl;
            cout << "Kinetic: " << scientific << kinEnergy << endl;
            cout << "Potential: " << scientific << potEnergy << endl << endl;
        }

        accel = accelNew;
    }
}


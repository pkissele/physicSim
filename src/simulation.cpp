#include "simulation.h"

#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include "utils.h"
#include "body.h"
#include "consts.h"

using namespace Consts;

using namespace std;

const double collRad = 0.01;
const double collMassRatio = 10;
const bool COLLISION_FLAG = true;

double gravEpsilon = 0.2;
double gravEpsilon2 = pow(gravEpsilon, 2);

double stepFrac = 1; // Janky sub-stepping

mt19937 rng(random_device{}());
uniform_real_distribution<double> randDist(0.0, 1.0);

using Vec2 = Eigen::Vector2d;

void NBodySimulation::deleteBody(int index) {
    if(index != aliveN - 1) {
        swap(bodies[index], bodies[aliveN - 1]);
    }

    aliveN += -1; //Move end pointer
}


void NBodySimulation::collide(int b1, int b2) {
    Body body1 = bodies[b1];
    Body body2 = bodies[b2];

    if(body1.mass < body2.mass) {
        swap(body1, body2);
        swap(b1, b2);
    }

    double dist = (body1.pos - body2.pos).norm();

    if((dist < collRad * (body1.size + body2.size)) && body1.mass > collMassRatio * body2.mass) {
        body1.vel = (body1.mass * body1.vel + body2.mass * body2.vel) / (body1.mass + body2.mass);
        body1.mass += body2.mass;
        deleteBody(b2);
    }
}


vector<Vec2> NBodySimulation::computeAccelerations(bool DO_INFO = false, double &potEnergy = *(new double)) {
    vector<Vec2> accel(N, Vec2(0.0, 0.0));
    if (DO_INFO) potEnergy = 0;

    for (int i = 0; i < aliveN; ++i) {
        for (int j = i+1; j < aliveN; ++j) {
            Vec2 dir = bodies[j].pos - bodies[i].pos;
            double dx = dir[0], dy = dir[1];
            double distSq = dx*dx + dy*dy;
            double r = sqrt(distSq + gravEpsilon2);
            double invDist = 1.0 / r;

            double gravMag = G * bodies[j].mass / (distSq + gravEpsilon2);
            accel[i][0] += gravMag * dx * invDist;
            accel[i][1] += gravMag * dy * invDist;

            accel[j] += -accel[i]/bodies[j].mass * bodies[i].mass;

            if (DO_INFO && i < j) {
                potEnergy += -1 * G * bodies[j].mass * bodies[i].mass / pow(distSq + gravEpsilon2, 0.5);
            }

            if(COLLISION_FLAG) {
                collide(i, j);
            }
        }
    }

    return accel;
}


NBodySimulation::NBodySimulation(int N_, double mass, double size, double viewW_, double viewH_)
    : N(N_), viewW(viewW_), viewH(viewH_) {

    aliveN = N;

    bodies.resize(aliveN);
    for(int i = 0; i < aliveN; ++i) {
        bodies[i].mass = mass;
        bodies[i].size = size;
    }

    // Create massive body
    bodies[0].mass = 10;
    bodies[0].size = 5 * size;
    bodies[1].mass = 10;
    bodies[1].size = 5 * size;

    Vec2 comPos = {0, 0};
    Vec2 comVel = {0, 0};
    double totMass = 0;

    double initVelScale = 1;
    double diskScale = 2; 

    diskScale = min(viewW, viewH)/2;

    for (int i = 0; i < aliveN; ++i) {
        if(i != 0) {
            randDisk(&bodies[i], diskScale, bodies[0].mass, mass, N);
        }
        bodies[i].vel *= initVelScale;
        // bodies[i].pos = randDisk() * min(viewW, viewH)/2;
        // bodies[i].pos = Vec2(randDist(rng) * viewW, randDist(rng) * viewH);
        comPos += bodies[i].pos * bodies[i].mass;
        totMass += bodies[i].mass;
    }

    accel = computeAccelerations();

    for (int i = 0; i < aliveN; ++i) {
        if(i != 0) {
            setOrbitalVel(&bodies[i], &accel[i], &bodies[0].pos);
            comVel += bodies[i].vel * bodies[i].mass;
        }
    }

    accel = computeAccelerations();

    comPos = comPos / totMass;
    comVel = comVel / totMass;

    cout << comPos << endl;
    Vec2 center = {viewW_/2, viewH_/2};
    // Center to avoid drift
    for (int i = 0; i < aliveN; ++i) {
        bodies[i].pos += -comPos + center;
        bodies[i].vel += -comVel;
    }

    // double meanSpacing = sqrt(viewW * viewH / N);
    // double gravEpsilon = 0.1 * meanSpacing;
    // double gravEpsilon1 = gravEpsilon * gravEpsilon;

}

void NBodySimulation::step(double dtIn, bool DO_INFO) {
    double dt = dtIn / stepFrac;
    for (int ss = 0; ss < stepFrac; ss++) {
        bool INFO_FLAG = (ss == 0 && DO_INFO);
        double kinEnergy = 0, potEnergy = 0;

        // half-step velocity
        for (int i = 0; i < aliveN; ++i) {
            bodies[i].vel += 0.5 * accel[i] * dt;
        }

        // full-step position
        for (int i = 0; i < aliveN; ++i) {
            bodies[i].pos += bodies[i].vel * dt;
        }

        // compute new accelerations
        vector<Vec2> accelNew;
        if (INFO_FLAG) accelNew = computeAccelerations(true, potEnergy);
        else accelNew = computeAccelerations();

        // complete velocity update
        for (int i = 0; i < aliveN; ++i) {
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


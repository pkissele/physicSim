#include "tree.h"

#include <random>
#include <vector>
#include <cmath>
#include <iostream>
// #include <iomanip>
#include <algorithm>
// #include <functional>

#include "utils.h"
#include "body.h"
#include "consts.h"
#include "node.h"

using namespace Consts;

using namespace std;

const double collRad = 0.01;
const double collMassRatio = 10;
const bool COLLISION_FLAG = true;

const double theta = 0.4;

double gravEpsilon = 0.005;
double gravEpsilon2 = pow(gravEpsilon, 2);

double stepFrac = 1; // Janky sub-stepping

mt19937 rng(random_device{}());
uniform_real_distribution<double> randDist(0.0, 1.0);

using Vec2 = Eigen::Vector2d;

quadTreeSim::quadTreeSim(int N_, double mass, double size, double viewW_, double viewH_)
    : N(N_), viewW(viewW_), viewH(viewH_) {

    aliveN = N;

    nodeCnt = 0;

    markDelete.reserve(aliveN);

    bodies.resize(aliveN);
    for(int i = 0; i < aliveN; ++i) {
        bodies[i].mass = mass;
        bodies[i].size = size;
    }

    // Create massive central body
    bodies[0].mass = 10;
    bodies[0].size = 5 * size;

    Vec2 comPos = {0, 0};
    Vec2 comVel = {0, 0};
    double totMass = 0;

    double initVelScale = 1;
    double diskScale = 1;

    double screenFracScale = 2.5;

    diskScale = min(viewW, viewH)/screenFracScale;

    for (int i = 0; i < aliveN; ++i) {
        if(i != 0) {
            randDisk(&bodies[i], diskScale, bodies[0].mass, mass, N);
        }
        bodies[i].vel *= initVelScale;
        comPos += bodies[i].pos * bodies[i].mass;
        totMass += bodies[i].mass;
    }

    buildTree();
    accel.resize(aliveN, Vec2(0.0, 0.0));
    double potEnergy;
    for(int i = 0; i < aliveN; ++i) {
        accel[i] = computeAccel(i, 0, &potEnergy);
    }

    // for (int i = 0; i < aliveN; ++i) {
    //     randVels(&bodies[i], diskScale/4);
    //     comVel += bodies[i].vel * bodies[i].mass;
    // }

    for (int i = 0; i < aliveN; ++i) {
        if(i != 0) {
            setOrbitalVel(&bodies[i], &accel[i], &bodies[0].pos);
            comVel += bodies[i].vel * bodies[i].mass;
        }
    }

    buildTree();
    for(int i = 0; i < aliveN; ++i) {
        accel[i] = computeAccel(i, 0, &potEnergy);
    }

    comPos = comPos / totMass;
    comVel = comVel / totMass;

    cout << comPos << endl;
    Vec2 center = {viewW_/2, viewH_/2};

    // Center to avoid drift
    for (int i = 0; i < aliveN; ++i) {
        bodies[i].pos += -comPos + center;
        bodies[i].vel += -comVel;
    }
}

void quadTreeSim::step(double dtIn, bool DO_INFO) {
    double dt = dtIn / stepFrac;
    for (int ss = 0; ss < stepFrac; ss++) {
        bool INFO_FLAG = (ss == 0 && DO_INFO);
        double kinEnergy = 0, potEnergy = 0;
        // half-step velocity (kick)
        for (int i = 0; i < aliveN; ++i) {
            bodies[i].vel += 0.5 * accel[i] * dt;
        }
        // full-step position (drift)
        for (int i = 0; i < aliveN; ++i) {
            bodies[i].pos += bodies[i].vel * dt;
        }

        // recompute acceleration
        buildTree();
        vector<Vec2> accelNew(aliveN, Vec2(0.0, 0.0));;
        for (int i = 0; i < aliveN; ++i) {
            accelNew[i] = computeAccel(i, 0, &potEnergy);
        }

        // half-step velocity (kick)
        for (int i = 0; i < aliveN; ++i) {
            bodies[i].vel += 0.5 * accelNew[i]*dt;
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

void quadTreeSim::buildTree() {
    Vec2 minCorner = {100000, 100000};
    Vec2 maxCorner = {-100000, -100000};

    tree.assign(aliveN * 8, Node());
    nodeCnt = 0;

    for (int i = 0; i < aliveN; ++i) {
        if(bodies[i].pos[0] < minCorner[0]) {
            minCorner[0] = bodies[i].pos[0];
        } else if(bodies[i].pos[0] > maxCorner[0]) {
            maxCorner[0] = bodies[i].pos[0];
        }
        if(bodies[i].pos[1] < minCorner[1]) {
            minCorner[1] = bodies[i].pos[1];
        } else if(bodies[i].pos[1] > maxCorner[1]) {
            maxCorner[1] = bodies[i].pos[1];
        }
    }

    Vec2 rootCenter = {(maxCorner[0]+minCorner[0])/2, (maxCorner[1]+minCorner[1])/2};
    double rootHalfSize = max(abs(maxCorner[0]-minCorner[0]), abs(maxCorner[1]-minCorner[1]))/2;

    tree[0] = Node();
    tree[0].center = rootCenter;
    tree[0].halfSize = rootHalfSize;
    nodeCnt += 1;

    for(int i = 0; i < aliveN; ++i) {
        insertParticle(0, i);
    }

    computeMassDistribution(0);
}

void quadTreeSim::insertParticle(int nInd, int bInd) {
    bool noChild = true;
    for(int i = 0; i < 4; i++) {
        if(tree[nInd].children[i] != -1) {
            noChild = false;
        }
    }
    if(noChild) {
        if(tree[nInd].bIndex == -1) {
            tree[nInd].bIndex = bInd;
        } else {
            int oldInd = tree[nInd].bIndex;
            tree[nInd].bIndex = -1;
            subdivide(nInd);
            insertParticle(nInd, oldInd);
            insertParticle(nInd, bInd);
        }
    } else {
        int quadrant = 0;
        if(bodies[bInd].pos[0] > tree[nInd].center[0]) quadrant += 1;
        if(bodies[bInd].pos[1] > tree[nInd].center[1]) quadrant += 2;

        insertParticle(tree[nInd].children[quadrant], bInd);
    }
}

void quadTreeSim::subdivide(int nInd) {
    for(int i = 0; i < 4; i++) {
        int quadrant = i;
        tree[nodeCnt] = Node();
        tree[nodeCnt].center = tree[nInd].center;
        tree[nodeCnt].halfSize = tree[nInd].halfSize/2;
        tree[nodeCnt].center[0] += (tree[nInd].halfSize/2) * (((int)quadrant % 2) == 1 ? 1 : -1);
        tree[nodeCnt].center[1] += (tree[nInd].halfSize/2) * (((int)quadrant / 2) == 1 ? 1 : -1);

        tree[nInd].children[quadrant] = nodeCnt;
        nodeCnt += 1;
    }
}

void quadTreeSim::computeMassDistribution(int nInd) {
    if(tree[nInd].bIndex != -1) {
        tree[nInd].mass = bodies[tree[nInd].bIndex].mass;
        tree[nInd].com = bodies[tree[nInd].bIndex].pos;
    } else {
        double totMass = 0;
        Vec2 weightSum = {0.0, 0.0};
        for(int i = 0; i < 4; i++) {
            if(tree[nInd].children[i] != -1) {
                computeMassDistribution(tree[nInd].children[i]);
                totMass += tree[tree[nInd].children[i]].mass;
                weightSum += tree[tree[nInd].children[i]].com * tree[tree[nInd].children[i]].mass;
            }
        }
        tree[nInd].mass = totMass;
        if(totMass != 0) {
            tree[nInd].com = weightSum / totMass;
        } else {
            tree[nInd].com = Vec2(0,0);
        }
    }
}

Vec2 quadTreeSim::computeAccel(int bInd, int nInd, double* potEnergy = nullptr) {
    if(bInd == tree[nInd].bIndex) return Vec2(0.0, 0.0);

    Vec2 r = tree[nInd].com - bodies[bInd].pos;
    double d = r.norm();
    double s = tree[nInd].halfSize * 2;
    
    if(tree[nInd].bIndex != -1 || (s/d) < theta) {
        double distSq = pow(d, 2) + gravEpsilon2;
        *potEnergy += -(double)1/2 * G * tree[nInd].mass * bodies[bInd].mass / sqrt(distSq);
        return G * tree[nInd].mass * r / (distSq*sqrt(distSq));
    } else {
        Vec2 totAccel = {0.0, 0.0};

        for(int i = 0; i < 4; i++) {
            if(tree[nInd].children[i] != -1) {
                totAccel += computeAccel(bInd, tree[nInd].children[i], potEnergy);
            }
        }
        return totAccel;
    }
}

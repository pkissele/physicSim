#include "tree.h"

#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "utils.h"
#include "body.h"
#include "consts.h"
#include "node.h"

using namespace Consts;

using namespace std;


// Fast square
template<typename T> inline constexpr T sq(T x) { return x * x; }


const double collRad = 0.01;
const double collMassRatio = 10;
const bool COLLISION_FLAG = true;

const double theta = 1;

double gravEpsilon = 0.003;
double gravEpsilon2 = pow(gravEpsilon, 2);

double stepFrac = 1; // Janky sub-stepping

mt19937 rng(random_device{}());
uniform_real_distribution<double> randDist(0.0, 1.0);

using Vec2 = Eigen::Vector2d;

quadTreeSim::quadTreeSim(int N_, double mass, double size, double viewW_, double viewH_)
    : N(N_), viewW(viewW_), viewH(viewH_) {

    aliveN = N;
    nodeCnt = 0;

    accel.resize(aliveN, Vec2(0.0, 0.0));
    accelNew.resize(aliveN, Vec2(0.0, 0.0));
    tree.resize(aliveN * 8);
    bodies.resize(aliveN);
    parents.reserve(aliveN);

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

    double screenFracScale = 2.5;
    double diskScale = min(viewW, viewH)/screenFracScale;;

    for (int i = 0; i < aliveN; ++i) {
        if(i != 0) {
            randDisk(&bodies[i], diskScale, bodies[0].mass, mass, N);
        }
        comPos += bodies[i].pos * bodies[i].mass;
        totMass += bodies[i].mass;
    }

    // Initialize into stable orbit
    buildTree();
    for(int i = 0; i < aliveN; ++i) {
        accel[i] = computeAccel(i, false, nullptr);
    }
    for (int i = 0; i < aliveN; ++i) {
        if(i != 0) {
            setOrbitalVel(&bodies[i], &accel[i], &bodies[0].pos);
            //     randVels(&bodies[i], diskScale/4);
            comVel += bodies[i].vel * bodies[i].mass;
        }
    }

    // Center to avoid drift
    comPos = comPos / totMass;
    comVel = comVel / totMass;
    Vec2 center = {viewW_/2, viewH_/2};
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
        for (int i = 0; i < aliveN; ++i) {
            accelNew[i] = computeAccel(i, DO_INFO, &potEnergy);
        }

        // half-step velocity (kick)
        for (int i = 0; i < aliveN; ++i) {
            bodies[i].vel += 0.5 * accelNew[i]*dt;
            if (INFO_FLAG) {
                kinEnergy += 0.5 * bodies[i].mass * (bodies[i].vel.squaredNorm());
            }
        }

        if (INFO_FLAG) {
            cout << "Energy: " << scientific << kinEnergy + potEnergy << endl;
            cout << "Kinetic: " << scientific << kinEnergy << endl;
            cout << "Potential: " << scientific << potEnergy << endl;
        }

        swap(accel, accelNew);
    }
}

void quadTreeSim::buildTree() {
    Vec2 minCorner = {100000, 100000};
    Vec2 maxCorner = {-100000, -100000};

    tree.assign(aliveN * 8, Node());
    parents.clear();
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

    computeMassDistribution();
}

void quadTreeSim::insertParticle(int nInd, int bInd) {
    if(tree[nInd].firstChild == -1) {
        if(tree[nInd].bIndex == -1) {
            tree[nInd].bIndex = bInd;
            tree[nInd].mass = bodies[bInd].mass;
            tree[nInd].com = bodies[bInd].pos;
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

        insertParticle(tree[nInd].firstChild+quadrant, bInd);
    }
}

void quadTreeSim::subdivide(int nInd) {
    parents.push_back(nInd);
    tree[nInd].firstChild = nodeCnt;
    for(int i = 0; i < 4; i++) {
        tree[nodeCnt+i] = Node();
        tree[nodeCnt+i].center = tree[nInd].center;
        tree[nodeCnt+i].halfSize = tree[nInd].halfSize/2;
        tree[nodeCnt+i].center[0] += (tree[nInd].halfSize/2) * (((int)i % 2) == 1 ? 1 : -1);
        tree[nodeCnt+i].center[1] += (tree[nInd].halfSize/2) * (((int)i / 2) == 1 ? 1 : -1);
        tree[nodeCnt+i].next = (i < 3) ? nodeCnt + i + 1 : tree[nInd].next;
    }
    nodeCnt += 4;
}


void quadTreeSim::computeMassDistribution() {
    static vector<int> stack;
    stack.clear();

    // process bottom up 
    for (int i = parents.size() - 1; i >= 0; i--) {
        int nInd = parents[i];
        if (tree[nInd].bIndex != -1) {
            tree[nInd].mass = bodies[tree[nInd].bIndex].mass;
            tree[nInd].com = bodies[tree[nInd].bIndex].pos;
        } else if(tree[nInd].firstChild != -1){
            double totMass = 0;
            Vec2 weightSum = {0.0, 0.0};
            for (int c = 0; c < 4; c++) {
                int child = tree[nInd].firstChild + c;
                totMass += tree[child].mass;
                weightSum += tree[child].com * tree[child].mass;
            }

            tree[nInd].mass = totMass;
            tree[nInd].com = totMass != 0 ? weightSum / totMass : Vec2(0, 0);
        }
    }
}


Vec2 quadTreeSim::computeAccel(int bInd, bool DO_INFO, double* potEnergy) {
    Vec2 totAccel = {0.0, 0.0};

    static vector<int> stack;
    stack.clear();
    stack.push_back(0);

    while (!stack.empty()) {
        int nInd = stack.back();
        stack.pop_back();

        if (bInd == tree[nInd].bIndex) continue;

        Vec2 r = tree[nInd].com - bodies[bInd].pos;
        double d = r.norm();
        double s = tree[nInd].halfSize * 2;

        if (tree[nInd].bIndex != -1 || (s/d) < theta) {
            double distSq = sq(d) + gravEpsilon2;
            double dist = sqrt(distSq);

            if (DO_INFO && potEnergy) *potEnergy += -0.5 * G * tree[nInd].mass * bodies[bInd].mass / dist;

            totAccel += G * tree[nInd].mass * r / (distSq * dist);
        } else if (tree[nInd].firstChild != -1){
            for (int i = 0; i < 4; i++) {
                stack.push_back(tree[nInd].firstChild + i);
            }
        }
    }
    return totAccel;
}

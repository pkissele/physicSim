#include "tree.h"

#include <cassert>
#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "utils.h"
#include "body.h"
#include "bodies.h"
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
const double init_theta = 0.5; // more accurate initialization

double gravEpsilon = 0.005;
double gravEpsilon2 = pow(gravEpsilon, 2);

double stepFrac = 1; // Janky sub-stepping

mt19937 rng(random_device{}());
uniform_real_distribution<double> randDist(0.0, 1.0);

using Vec2 = Eigen::Vector2d;


quadTreeSim::quadTreeSim(int N_, double mass, double size, double viewW_, double viewH_)
    : N(N_), viewW(viewW_), viewH(viewH_), bodies(N_){

    aliveN = N;
    nodeCnt = 0;

    accel.resize(aliveN, Vec2(0.0, 0.0));
    accelNew.resize(aliveN, Vec2(0.0, 0.0));
    tree.resize(aliveN * 8);

    parents.reserve(aliveN);

    for(int i = 0; i < aliveN; ++i) {
        bodies.mass[i] = mass;
        bodies.size[i] = size;
    }



    double screenFracScale = 2.5;
    double diskScale = min(viewW, viewH)/screenFracScale;;


    randDisk(&bodies, diskScale);

    // Create massive central body
    bodies.mass[0] = 10;
    bodies.size[0] = 5 * size;
    bodies.px[0] = 0.0f;
    bodies.py[0] = 0.0f;

    double comPx = 0.0;
    double comPy = 0.0;
    double totMass = 0;

    for (int i = 0; i < aliveN; ++i) {
        comPx += bodies.px[i] * bodies.mass[i];
        comPy += bodies.py[i] * bodies.mass[i];
        totMass += bodies.mass[i];
    }

    bodies.vx[0] = 0.0f;
    bodies.vy[0] = 0.0f;

    // Initialize into stable orbit
    buildTree();
    for(int i = 0; i < aliveN; ++i) {
        accel[i] = computeAccel(i, init_theta, false, nullptr);
    }
    double comVx = 0.0;
    double comVy = 0.0;
    Vec2 centPos(bodies.px[0], bodies.py[0]);
    setOrbitalVel(&bodies, accel, centPos);
    for (int i = 0; i < aliveN; ++i) {
        comVx += bodies.vx[i] * bodies.mass[i];
        comVy += bodies.vy[i] * bodies.mass[i];
    }

    // Center to avoid drift
    comPx = comPx / totMass;
    comPy = comPy / totMass;
    comVx = comVx / totMass;
    comVy = comVy / totMass;
    Vec2 center = {viewW_/2, viewH_/2};
    for (int i = 0; i < aliveN; ++i) {
        bodies.px[i] += -comPx + center[0];
        bodies.py[i] += -comPy + center[1];
        bodies.vx[i] += -comVx;
        bodies.vy[i] += -comVy;
    }
}

void quadTreeSim::step(double dtIn, bool DO_INFO) {
    double dt = dtIn / stepFrac;
    for (int ss = 0; ss < stepFrac; ss++) {
        bool INFO_FLAG = (ss == 0 && DO_INFO);
        double kinEnergy = 0, potEnergy = 0;

        // half-step velocity (kick)
        for (int i = 0; i < aliveN; ++i) {
            bodies.vx[i] += 0.5 * accel[i][0] * dt;
            bodies.vy[i] += 0.5 * accel[i][1] * dt;
        }

        // full-step position (drift)
        for (int i = 0; i < aliveN; ++i) {
            bodies.px[i] += bodies.vx[i] * dt;
            bodies.py[i] += bodies.vy[i] * dt;
        }

        // recompute acceleration
        buildTree();
        auto start = chrono::high_resolution_clock::now();
        for (int i = 0; i < aliveN; ++i) {
            accelNew[i] = computeAccel(i, theta, DO_INFO, &potEnergy);
        }
        auto end = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double, milli>(end - start).count();
        cout << "accel step took "<< fixed << elapsed << " ms" << endl << endl;

        // half-step velocity (kick)
        for (int i = 0; i < aliveN; ++i) {
            bodies.vx[i] += 0.5 * accelNew[i][0] * dt;
            bodies.vy[i] += 0.5 * accelNew[i][1] * dt;
            if (INFO_FLAG) {
                kinEnergy += 0.5 * bodies.mass[i] * ((sq(bodies.vx[i])+sq(bodies.vy[i])));
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

    for (int i = 0; i < aliveN; i++) {
        minCorner[0] = min(minCorner[0], (double)bodies.px[i]);
        maxCorner[0] = max(maxCorner[0], (double)bodies.px[i]);
        minCorner[1] = min(minCorner[1], (double)bodies.py[i]);
        maxCorner[1] = max(maxCorner[1], (double)bodies.py[i]);
    }

    Vec2 rootCenter = {(maxCorner[0]+minCorner[0])/2, (maxCorner[1]+minCorner[1])/2};
    double rootHalfSize = max(abs(maxCorner[0]-minCorner[0]), abs(maxCorner[1]-minCorner[1]))/2;

    tree[0] = Node();
    tree[0].center = rootCenter;
    tree[0].halfSize = rootHalfSize * 1.001;
    nodeCnt += 1;

    for(int i = 0; i < aliveN; ++i) {
        insertParticle(i);
    }

    computeMassDistribution();
}


int quadTreeSim::getQuadrant(int bInd, int nInd) {
    Node n = tree[nInd];
    int q = 0;
    if (bodies.px[bInd] > n.center[0]) q += 1;
    if (bodies.py[bInd] > n.center[1]) q += 2;
    return q;
}


void quadTreeSim::insertParticle(int bInd) {
    // Start from root
    int nInd = 0;

    // descend to leaf
    while (tree[nInd].firstChild != -1) {
        nInd = tree[nInd].firstChild + getQuadrant(bInd, nInd);
    }

    // empty leaf
    if (tree[nInd].bIndex == -1) {
        tree[nInd].bIndex = bInd;
        tree[nInd].mass = bodies.mass[bInd];
        tree[nInd].com[0] = bodies.px[bInd];
        tree[nInd].com[1] = bodies.py[bInd];
        return;
    }

    // duplicate position
    int oldInd = tree[nInd].bIndex;
    if (bodies.px[bInd] == bodies.px[oldInd] && bodies.py[bInd] == bodies.py[oldInd]) {
        tree[nInd].mass += bodies.mass[bInd];
        return;
    }

    // subdivide until separated
    tree[nInd].bIndex = -1;
    tree[nInd].mass = 0;
    tree[nInd].com = Vec2(0,0);

    while (true) {
        subdivide(nInd);

        int quadOld = getQuadrant(oldInd, nInd);
        int quadNew = getQuadrant(bInd, nInd);

        if (quadOld != quadNew) {
            int oldChild = tree[nInd].firstChild + quadOld;
            tree[oldChild].bIndex = oldInd;
            tree[oldChild].mass = bodies.mass[oldInd];
            tree[oldChild].com[0] = bodies.px[oldInd];
            tree[oldChild].com[1] = bodies.py[oldInd];

            int newChild = tree[nInd].firstChild + quadNew;
            tree[newChild].bIndex = bInd;
            tree[newChild].mass = bodies.mass[bInd];
            tree[newChild].com[0] = bodies.px[bInd];
            tree[newChild].com[1] = bodies.py[bInd];
            return;
        }

        nInd = tree[nInd].firstChild + quadOld;
    }
}


void quadTreeSim::subdivide(int nInd) {
    parents.push_back(nInd);
    tree[nInd].firstChild = nodeCnt;
    for(int i = 0; i < 4; i++) {
        tree[nodeCnt+i] = Node();
        tree[nodeCnt+i].center = tree[nInd].center;
        tree[nodeCnt+i].halfSize = tree[nInd].halfSize/2;
        tree[nodeCnt+i].center[0] += (tree[nInd].halfSize/2) * ((i % 2) == 1 ? 1 : -1);
        tree[nodeCnt+i].center[1] += (tree[nInd].halfSize/2) * ((i / 2) == 1 ? 1 : -1);
        tree[nodeCnt+i].next = (i < 3) ? nodeCnt + i + 1 : tree[nInd].next;
    }
    nodeCnt += 4;
}


void quadTreeSim::computeMassDistribution() {
    // process bottom up 
    for (int i = parents.size() - 1; i >= 0; i--) {
        int nInd = parents[i];
        if(tree[nInd].firstChild == -1) continue;

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


Vec2 quadTreeSim::computeAccel(int bInd, double thetaIn, bool DO_INFO, double* potEnergy) {
    Vec2 totAccel = {0.0, 0.0};
    int nInd = 0;

    // Go from root down through child and subtree until good enough
    while (nInd != -1) {
        const Node& node = tree[nInd];

        if (node.mass == 0.0) { nInd = node.next; continue; }
        if (bInd == node.bIndex) { nInd = node.next; continue; }

        double s = node.halfSize * 2;
        Vec2 rGeom(node.center[0] - bodies.px[bInd], node.center[1] - bodies.py[bInd]);
        double distSqGeom = rGeom.squaredNorm();

        // monopole criterion
        if (node.firstChild == -1 || sq(s) < sq(thetaIn) * distSqGeom) {
            Vec2 r(node.com[0] - bodies.px[bInd], node.com[1] - bodies.py[bInd]);
            double distSq = r.squaredNorm();
            double softDistSq = distSq + gravEpsilon2;
            double invDist = 1.0 / sqrt(softDistSq);
            totAccel += G * node.mass * r * (invDist * invDist * invDist);
            nInd = node.next;
            if (DO_INFO && potEnergy) *potEnergy += -0.5 * G * node.mass * bodies.mass[bInd] * invDist;
        } else {
            nInd = node.firstChild;
        }
    }
    return totAccel;
}

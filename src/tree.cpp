#include "tree.h"

#include <cassert>
#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>

#include "utils.h"
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

const float theta = 1;
const float init_theta = 0.5; // more accurate initialization

double gravEpsilon = 0.005;
double gravEpsilon2 = sq(gravEpsilon);

double stepFrac = 1; // Janky sub-stepping

mt19937 rng(random_device{}());
uniform_real_distribution<double> randDist(0.0, 1.0);


quadTreeSim::quadTreeSim(int N_, double mass, double size, double viewW_, double viewH_)
    : N(N_), viewW(viewW_), viewH(viewH_), bodies(N_){


    cout << sizeof(Node) << endl;

    nodeCnt = 0;
    tree.resize(N * 8);

    parents.reserve(N);

    for(int i = 0; i < N; ++i) {
        bodies.mass[i] = mass;
        bodies.size[i] = size;
    }

    double screenFracScale = 2.5;
    double diskScale = min(viewW, viewH)/screenFracScale;;

    randDisk(bodies, diskScale);

    // Create massive central body
    bodies.mass[0] = 10;
    bodies.size[0] = 5 * size;
    bodies.px[0] = 0.0f;
    bodies.py[0] = 0.0f;

    double comPx = 0.0;
    double comPy = 0.0;
    double totMass = 0;

    for (int i = 0; i < N; ++i) {
        comPx += bodies.px[i] * bodies.mass[i];
        comPy += bodies.py[i] * bodies.mass[i];
        totMass += bodies.mass[i];
    }

    bodies.vx[0] = 0.0f;
    bodies.vy[0] = 0.0f;

    // Initialize into stable orbit
    buildTree();
    for(int i = 0; i < N; ++i) {
        computeAccel(i, init_theta, bodies.ax, bodies.ay, false, nullptr);
    }

    float comVx = 0.0;
    float comVy = 0.0;
    setOrbitalVel(bodies, bodies.px[0], bodies.py[0]);
    for (int i = 0; i < N; ++i) {
        comVx += bodies.vx[i] * bodies.mass[i];
        comVy += bodies.vy[i] * bodies.mass[i];
    }

    // Center to avoid drift
    comPx = comPx / totMass;
    comPy = comPy / totMass;
    comVx = comVx / totMass;
    comVy = comVy / totMass;
    float centX = viewW_/2;
    float centY = viewH_/2;
    for (int i = 0; i < N; ++i) {
        bodies.px[i] += -comPx + centX;
        bodies.py[i] += -comPy + centY;
        bodies.vx[i] += -comVx;
        bodies.vy[i] += -comVy;
    }
}


void quadTreeSim::step(double dtIn, bool DO_INFO) {
    float dt = dtIn / stepFrac;
    for (int ss = 0; ss < stepFrac; ss++) {
        bool INFO_FLAG = (ss == 0 && DO_INFO);
        double kinEnergy = 0, potEnergy = 0;

        // half-step velocity (kick)
        for (int i = 0; i < N; ++i) {
            bodies.vx[i] += 0.5f * bodies.ax[i] * dt;
            bodies.vy[i] += 0.5f * bodies.ay[i] * dt;
        }

        // full-step position (drift)
        for (int i = 0; i < N; ++i) {
            bodies.px[i] += bodies.vx[i] * dt;
            bodies.py[i] += bodies.vy[i] * dt;
        }

        // recompute acceleration
        buildTree();
        auto start = chrono::high_resolution_clock::now();
        for (int i = 0; i < N; ++i) {
            computeAccel(i, theta, bodies.axNew, bodies.ayNew, DO_INFO, &potEnergy);
        }
        auto end = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double, milli>(end - start).count();
        cout << "accel step took "<< fixed << elapsed << " ms" << endl << endl;

        // half-step velocity (kick)
        for (int i = 0; i < N; ++i) {
            bodies.vx[i] += 0.5f * bodies.axNew[i] * dt;
            bodies.vy[i] += 0.5f * bodies.ayNew[i] * dt;
            if (INFO_FLAG) {
                kinEnergy += 0.5f * bodies.mass[i] * ((sq(bodies.vx[i])+sq(bodies.vy[i])));
            }
        }

        if (INFO_FLAG) {
            cout << "Energy: " << scientific << kinEnergy + potEnergy << endl;
            cout << "Kinetic: " << scientific << kinEnergy << endl;
            cout << "Potential: " << scientific << potEnergy << endl;
        }

        swap(bodies.ax, bodies.axNew);
        swap(bodies.ay, bodies.ayNew);
    }
}



void quadTreeSim::buildTree() {
    float minX =  1e9; float minY =  1e9;
    float maxX = -1e9; float maxY = -1e9; 

    fill(tree.begin(), tree.begin() + nodeCnt, Node());

    parents.clear();
    nodeCnt = 0;

    for (int i = 0; i < N; i++) {
        minX = min(minX, bodies.px[i]);
        maxX = max(maxX, bodies.px[i]);
        minY = min(minY, bodies.py[i]);
        maxY = max(maxY, bodies.py[i]);
    }

    float rootCenterX = (maxX + minX)/2.0f;
    float rootCenterY = (maxY + minY)/2.0f;
    float rootHalfSize = max(abs(maxX - minX), abs(maxY - minY))/2;

    tree[0] = Node();
    tree[0].cx = rootCenterX;
    tree[0].cy = rootCenterY;
    tree[0].halfSize = rootHalfSize;
    nodeCnt += 1;

    for(int i = 0; i < N; ++i) {
        insertParticle(i);
    }

    computeMassDistribution();
}


int quadTreeSim::getQuadrant(int bInd, int nInd) {
    const Node& n = tree[nInd];
    int q = 0;
    if (bodies.px[bInd] > n.cx) q += 1;
    if (bodies.py[bInd] > n.cy) q += 2;
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
        tree[nInd].comx = bodies.px[bInd];
        tree[nInd].comy = bodies.py[bInd];
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
    tree[nInd].comx = 0.0f;
    tree[nInd].comy = 0.0f;

    while (true) {
        subdivide(nInd);

        int quadOld = getQuadrant(oldInd, nInd);
        int quadNew = getQuadrant(bInd, nInd);

        if (quadOld != quadNew) {
            int oldChild = tree[nInd].firstChild + quadOld;
            tree[oldChild].bIndex = oldInd;
            tree[oldChild].mass = bodies.mass[oldInd];
            tree[oldChild].comx = bodies.px[oldInd];
            tree[oldChild].comy = bodies.py[oldInd];

            int newChild = tree[nInd].firstChild + quadNew;
            tree[newChild].bIndex = bInd;
            tree[newChild].mass = bodies.mass[bInd];
            tree[newChild].comx = bodies.px[bInd];
            tree[newChild].comy = bodies.py[bInd];
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
        tree[nodeCnt+i].cx = tree[nInd].cx;
        tree[nodeCnt+i].cy = tree[nInd].cy;
        tree[nodeCnt+i].halfSize = tree[nInd].halfSize/2;
        tree[nodeCnt+i].cx += (tree[nInd].halfSize/2) * ((i % 2) == 1 ? 1 : -1);
        tree[nodeCnt+i].cy += (tree[nInd].halfSize/2) * ((i / 2) == 1 ? 1 : -1);
        tree[nodeCnt+i].next = (i < 3) ? nodeCnt + i + 1 : tree[nInd].next;
    }
    nodeCnt += 4;
}


void quadTreeSim::computeMassDistribution() {
    // process bottom up 
    for (int i = parents.size() - 1; i >= 0; i--) {
        int nInd = parents[i];
        if(tree[nInd].firstChild == -1) continue;

        float totMass = 0;
        float weightSumX = 0.0f;
        float weightSumY = 0.0f;

        for (int c = 0; c < 4; c++) {
            int child = tree[nInd].firstChild + c;
            totMass += tree[child].mass;
            weightSumX += tree[child].comx * tree[child].mass;
            weightSumY += tree[child].comy * tree[child].mass;
        }

        tree[nInd].mass = totMass;
        tree[nInd].comx = totMass != 0.0f ? weightSumX / totMass : 0.0f;
        tree[nInd].comy = totMass != 0.0f ? weightSumY / totMass : 0.0f;
    }
}


void quadTreeSim::computeAccel(int bInd, float thetaIn, vector<float>& axOut, vector<float>& ayOut, bool DO_INFO, double* potEnergy) {
    float totAccX = 0.0f;
    float totAccY = 0.0f;
    int nInd = 0;

    float flG = (float)G;
    float flGravEpsilon2 = (float)gravEpsilon2;

    // Go from root down through child and subtree until good enough
    while (nInd != -1) {
        const Node& node = tree[nInd];

        if (node.mass == 0.0) { nInd = node.next; continue; }
        if (bInd == node.bIndex) { nInd = node.next; continue; }

        float s = node.halfSize * 2;
        float rGeomX = node.cx - bodies.px[bInd];
        float rGeomY = node.cy - bodies.py[bInd];
        float distSqGeom = sq(rGeomX) + sq(rGeomY);

        // monopole criterion
        if (node.firstChild == -1 || sq(s) < sq(thetaIn) * distSqGeom) {
            float rX = node.comx - bodies.px[bInd];
            float rY = node.comy - bodies.py[bInd];

            float distSq = sq(rX) + sq(rY);
            float softDistSq = distSq + flGravEpsilon2;
            float invDist = 1.0f / sqrt(softDistSq);
            float accelMag = flG * node.mass * (sq(invDist) * invDist);

            totAccX += accelMag * rX;
            totAccY += accelMag * rY;
            nInd = node.next;
            if (DO_INFO && potEnergy) *potEnergy += -0.5 * G * node.mass * bodies.mass[bInd] * invDist;
        } else {
            nInd = node.firstChild;
        }
    }
    axOut[bInd] = totAccX;
    ayOut[bInd] = totAccY;
}

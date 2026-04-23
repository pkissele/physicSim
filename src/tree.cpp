#include "tree.h"

#include <cassert>
#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <execution>

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

double gravEpsilon = 0.01;
// double gravEpsilon = 0.01;
double gravEpsilon2 = sq(gravEpsilon);

double stepFrac = 1; // Janky sub-stepping

const int LEAF_CAPACITY = 4;
const int THREAD_CAPACITY = 4096; 


void quadTreeSim::step(double dtIn, bool LOG_ENERGY, bool LOG_TIME) {
    float dt = dtIn / stepFrac;
    for (int ss = 0; ss < stepFrac; ss++) {
        bool INFO_FLAG = (ss == 0 && LOG_ENERGY);
        double kinEnergy = 0, potEnergy = 0;

        // half-step velocity (kick)
        for (int i = 0; i < N; i++) {
            bodies.vx[i] += 0.5f * bodies.ax[i] * dt;
            bodies.vy[i] += 0.5f * bodies.ay[i] * dt;
        }

        // full-step position (drift)
        for (int i = 0; i < N; i++) {
            bodies.px[i] += bodies.vx[i] * dt;
            bodies.py[i] += bodies.vy[i] * dt;
        }


        auto start = chrono::high_resolution_clock::now();
        buildTree();
        auto end = chrono::high_resolution_clock::now();
        double elapsed = chrono::duration<double, milli>(end - start).count();
        if(LOG_TIME) cout << "build tree took "<< fixed << elapsed << " ms" << endl;


        start = chrono::high_resolution_clock::now();
        #pragma omp parallel for schedule(dynamic, 64) reduction(+:potEnergy)
        for (int i = 0; i < N; i++) {
            double localPE = 0.0;
            computeAccel(i, theta, bodies.axNew, bodies.ayNew, LOG_ENERGY,
                        LOG_ENERGY ? &localPE : nullptr);
            potEnergy += localPE;
        }
        end = chrono::high_resolution_clock::now();
        elapsed = chrono::duration<double, milli>(end - start).count();
        if(LOG_TIME) cout << "accel step took "<< fixed << elapsed << " ms" << endl;

        // half-step velocity (kick)
        for (int i = 0; i < N; i++) {
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
    float minX =  1e9, minY =  1e9;
    float maxX = -1e9, maxY = -1e9;

    fill(tree.begin(), tree.begin() + nodeCnt, Node());
    nodeCnt = 0;

    for (int i = 0; i < N; i++) {
        minX = min(minX, bodies.px[i]);
        maxX = max(maxX, bodies.px[i]);
        minY = min(minY, bodies.py[i]);
        maxY = max(maxY, bodies.py[i]);
    }

    std::iota(buildIndices.begin(), buildIndices.end(), 0);

    tree[0] = Node();
    tree[0].cx = (maxX + minX) / 2.0f;
    tree[0].cy = (maxY + minY) / 2.0f;
    tree[0].halfSize = max(abs(maxX - minX), abs(maxY - minY)) / 2.0f;
    tree[0].lo = 0;
    tree[0].hi = N;
    tree[0].next = -1;
    nodeCnt = 1;

    #pragma omp parallel
    {
        #pragma omp single
        buildSubtree(0, 0);
    }

    reorderBodies();
    computeMassDistribution();
}


void quadTreeSim::buildSubtree(int nInd, int depth) {
    int lo = tree[nInd].lo;
    int hi = tree[nInd].hi;
    int count = hi - lo;

    auto makeLeaf = [&]() {
        float m = 0, wx = 0, wy = 0;
        for (int ii = lo; ii < hi; ii++) {
            int i = buildIndices[ii];
            m  += bodies.mass[i];
            wx += bodies.mass[i] * bodies.px[i];
            wy += bodies.mass[i] * bodies.py[i];
        }
        tree[nInd].mass = m;
        tree[nInd].comx = m > 0 ? wx / m : 0;
        tree[nInd].comy = m > 0 ? wy / m : 0;
    };

    if (count <= LEAF_CAPACITY || depth > 52) {
        makeLeaf();
        return;
    }

    float firstX = bodies.px[buildIndices[lo]];
    float firstY = bodies.py[buildIndices[lo]];
    bool allSame = true;
    for (int ii = lo + 1; ii < hi; ii++) {
        int i = buildIndices[ii];
        if (bodies.px[i] != firstX || bodies.py[i] != firstY) {
            allSame = false;
            break;
        }
    }
    if (allSame) {
        makeLeaf();
        return;
    }

    float cx = tree[nInd].cx;
    float cy = tree[nInd].cy;
    int* idx = buildIndices.data();

    auto midIt = std::partition(idx + lo, idx + hi, [&](int i) {return bodies.py[i] < cy;});
    int mid = midIt - idx;
    auto q01It = std::partition(idx + lo,  idx + mid, [&](int i) {return bodies.px[i] < cx;});
    auto q23It = std::partition(idx + mid, idx + hi,  [&](int i) {return bodies.px[i] < cx;});
    int splits[5] = { lo, (int)(q01It - idx), mid, (int)(q23It - idx), hi };

    subdivide(nInd);
    int fc = tree[nInd].firstChild;

    for (int q = 0; q < 4; q++) {
        int child = fc + q;
        tree[child].lo = splits[q];
        tree[child].hi = splits[q + 1];

        int cnt = splits[q+1] - splits[q];
        if (cnt >= THREAD_CAPACITY) {
            #pragma omp task firstprivate(child, depth) default(shared)
            buildSubtree(child, depth + 1);
        } else {
            buildSubtree(child, depth + 1);
        }
    }
}



int quadTreeSim::getQuadrant(int bInd, int nInd) {
    const Node& n = tree[nInd];
    int q = 0;
    if (bodies.px[bInd] > n.cx) q += 1;
    if (bodies.py[bInd] > n.cy) q += 2;
    return q;
}


void quadTreeSim::reorderBodies() {
    static std::vector<float> tmp;
    tmp.resize(N);

    auto reorderArray = [&](std::vector<float>& arr) {
        #pragma omp parallel for
        for (int i = 0; i < N; i++) tmp[i] = arr[buildIndices[i]];
        std::swap(arr, tmp);
    };

    reorderArray(bodies.px);
    reorderArray(bodies.py);
    reorderArray(bodies.vx);
    reorderArray(bodies.vy);
    reorderArray(bodies.ax);
    reorderArray(bodies.ay);
    reorderArray(bodies.axNew);
    reorderArray(bodies.ayNew);
    reorderArray(bodies.mass);
    reorderArray(bodies.size);

    std::iota(buildIndices.begin(), buildIndices.end(), 0);
}


void quadTreeSim::subdivide(int nInd) {
    int base = nodeCnt.fetch_add(4, std::memory_order_relaxed);
    assert(base + 4 <= (int)tree.size());
    tree[nInd].firstChild = base;
    for (int i = 0; i < 4; i++) {
        tree[base+i] = Node();
        tree[base+i].cx = tree[nInd].cx + (tree[nInd].halfSize/2) * ((i & 1) ? 1 : -1);
        tree[base+i].cy = tree[nInd].cy + (tree[nInd].halfSize/2) * ((i & 2) ? 1 : -1);
        tree[base+i].halfSize = tree[nInd].halfSize / 2;
        tree[base+i].next = (i < 3) ? base + i + 1 : tree[nInd].next;
    }
}


void quadTreeSim::computeMassDistribution() {
    for (int i = nodeCnt - 1; i >= 0; i--) {
        if (tree[i].firstChild == -1) continue;

        float totMass = 0, weightSumX = 0, weightSumY = 0;
        for (int c = 0; c < 4; c++) {
            int child = tree[i].firstChild + c;
            totMass     += tree[child].mass;
            weightSumX  += tree[child].comx * tree[child].mass;
            weightSumY  += tree[child].comy * tree[child].mass;
        }
        tree[i].mass = totMass;
        tree[i].comx = totMass > 0 ? weightSumX / totMass : 0;
        tree[i].comy = totMass > 0 ? weightSumY / totMass : 0;
    }
}


void quadTreeSim::computeAccel(int bInd, float thetaIn, vector<float>& axOut, vector<float>& ayOut, bool DO_INFO, double* potEnergy) {
    float totAccX = 0.0f, totAccY = 0.0f;
    int nInd = 0;
    float flG = (float)G;
    float flGravEpsilon2 = (float)gravEpsilon2;
    float bx = bodies.px[bInd];
    float by = bodies.py[bInd];

    while (nInd != -1) {
        const Node& node = tree[nInd];

        float rX = node.comx - bx;
        float rY = node.comy - by;
        float distSq = sq(rX) + sq(rY);
        float s = node.halfSize * 2.0f;

        bool isLeaf = (node.firstChild == -1);
        bool macPasses = sq(s) < sq(thetaIn) * distSq;
        bool selfInLeaf = isLeaf && bInd >= node.lo && bInd < node.hi;

        if (macPasses && !selfInLeaf) {
            float distSqSoft = distSq + flGravEpsilon2;
            float invDist = 1.0f / sqrt(distSqSoft);
            float accelMag = flG * node.mass * sq(invDist) * invDist;
            totAccX += accelMag * rX;
            totAccY += accelMag * rY;
            if (DO_INFO && potEnergy) *potEnergy += -0.5 * G * node.mass * bodies.mass[bInd] * invDist;
            nInd = node.next;
        } else if (isLeaf) {
            for (int j = node.lo; j < node.hi; j++) {
                if (j == bInd) continue;
                float dX = bodies.px[j] - bx;
                float dY = bodies.py[j] - by;
                float dSq = sq(dX) + sq(dY) + flGravEpsilon2;
                float invDist = 1.0f / sqrt(dSq);
                float accelMag = flG * bodies.mass[j] * sq(invDist) * invDist;
                totAccX += accelMag * dX;
                totAccY += accelMag * dY;
                if (DO_INFO && potEnergy)
                    *potEnergy += -0.5 * G * bodies.mass[j] * bodies.mass[bInd] * invDist;
            }
            nInd = node.next;
        } else {
            nInd = node.firstChild;
        }
    }

    axOut[bInd] = totAccX;
    ayOut[bInd] = totAccY;
}



quadTreeSim::quadTreeSim(int N_, double mass, double size, double viewW_, double viewH_)
    : N(N_), viewW(viewW_), viewH(viewH_), bodies(N_){
    nodeCnt = 0;

    tree.resize(N * 8);

    buildIndices.resize(N);
    std::iota(buildIndices.begin(), buildIndices.end(), 0);

    for(int i = 0; i < N; i++) {
        bodies.mass[i] = mass;
        bodies.size[i] = size;
    }

    double screenFracScale = 2.5;
    double diskScale = min(viewW, viewH)/screenFracScale;;

    randDisk(bodies, diskScale, 0.3);
    // randDisk(bodies, diskScale, 0);

    // Create massive central body
    bodies.mass[0] = 20;
    bodies.size[0] = 5 * size;
    bodies.px[0] = 0.0f;
    bodies.py[0] = 0.0f;

    // Initialize into stable orbit
    buildTree();
    for(int i = 0; i < N; i++) {
        computeAccel(i, init_theta, bodies.ax, bodies.ay, false, nullptr);
    }
    setOrbitalVel(bodies, 0, 0);
    bodies.vx[0] = 0;
    bodies.vy[0] = 0;

    double comPx = 0.0;
    double comPy = 0.0;
    double comVx = 0.0;
    double comVy = 0.0;
    double totMass = 0;
    for (int i = 0; i < N; i++) {
        comPx += bodies.px[i] * bodies.mass[i];
        comPy += bodies.py[i] * bodies.mass[i];
        comVx += bodies.vx[i] * bodies.mass[i];
        comVy += bodies.vy[i] * bodies.mass[i];
        totMass += bodies.mass[i];
    }

    // Center to avoid drift
    comPx = comPx / totMass;
    comPy = comPy / totMass;
    comVx = comVx / totMass;
    comVy = comVy / totMass;
    float centX = viewW_/2;
    float centY = viewH_/2;
    for (int i = 0; i < N; i++) {
        bodies.px[i] += -comPx + centX;
        bodies.py[i] += -comPy + centY;
        bodies.vx[i] += -comVx;
        bodies.vy[i] += -comVy;
    }

    // Remove central body velocity
    int centralIdx = 0;
    for (int i = 0; i < N; i++) {
        if (bodies.mass[i] > 1.0f) {
            centralIdx = i;
            break;
        }
    }
    bodies.vx[centralIdx] = 0;
    bodies.vy[centralIdx] = 0;
}



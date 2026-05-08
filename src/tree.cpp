#include "tree.h"

#include <cassert>
#include <numeric>
#include <random>
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <execution>

#include <Eigen/Dense>

#include "utils.h"
#include "bodies.h"
#include "node.h"
#include "timer.h"

#include "consts.h"


using namespace Consts;
using namespace std;



void quadTreeSim::step(double dt, bool LOG_ENERGY, bool LOG_TIME) {
    // double kinEnergy = 0, potEnergy = 0;

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

    {
        ScopedTimer t("build tree", LOG_TIME);
        buildTree();
    }
    {
        ScopedTimer t("accel step", LOG_TIME);
        #pragma omp parallel
        {
            #pragma omp single nowait
            computeAccelSubtree(0);
        }
    }

    for (int i = 0; i < N; i++) {
        findDensity(i);
    }

    // half-step velocity (kick)
    for (int i = 0; i < N; i++) {
        bodies.vx[i] += 0.5f * bodies.axNew[i] * dt;
        bodies.vy[i] += 0.5f * bodies.ayNew[i] * dt;
        // if (LOG_ENERGY) {
        //     kinEnergy += 0.5f * bodies.mass[i] * ((sq(bodies.vx[i])+sq(bodies.vy[i])));
        // }
    }
    //
    // if (LOG_ENERGY) {
    //     cout << "Energy: " << scientific << kinEnergy + potEnergy << endl;
    //     cout << "Kinetic: " << scientific << kinEnergy << endl;
    //     cout << "Potential: " << scientific << potEnergy << endl;
    // }
    swap(bodies.ax, bodies.axNew);
    swap(bodies.ay, bodies.ayNew);
}


void quadTreeSim::findDensity(int bInd) {
    int leafInd = bodyToLeaf[bInd];
    // Use containing leaf's size as a proxy for h
    // Kernel compact support is 2h, so this gives support ≈ leaf side length
    float h = tree[leafInd].size * 0.5f;
    if (h < 1e-8f) { bodies.dens[bInd] = 0.0f; return; }
    if (bodies.diffuse[bInd] == 0) { bodies.dens[bInd] = 0.0f; return; } 

    float invH = 1.0f / h;
    float invH2 = invH * invH;
    float kernelNorm = (7.0f / (4.0f * (float)M_PI)) * invH2;  // 2D Wendland C2 norm
    float searchR2 = sq(2.0f * h);

    float bX = bodies.px[bInd];
    float bY = bodies.py[bInd];
    float densAccum = 0.0f;

    // Range query: walk tree, summing kernel contributions within 2h
    int nInd = 0;
    while (nInd != -1) {
        const Node& node = tree[nInd];
        float cx = treeCold[nInd].cx;
        float cy = treeCold[nInd].cy;
        float halfSize = node.size * 0.5f;
        float dxN = std::max(0.0f, std::abs(bX - cx) - halfSize);
        float dyN = std::max(0.0f, std::abs(bY - cy) - halfSize);
        float minDist2 = sq(dxN) + sq(dyN);

        if (minDist2 > searchR2) {
            nInd = node.next;  // entire subtree outside support — skip
        } else if (node.firstChild == -1) {
            for (int j = node.lo; j < node.hi; j++) {
                float dx = bodies.px[j] - bX;
                float dy = bodies.py[j] - bY;
                float d2 = sq(dx) + sq(dy);
                if (d2 < searchR2) {
                    if (bodies.diffuse[j] == 0) continue; 
                    float q = std::sqrt(d2) * invH;
                    float t = 1.0f - 0.5f * q;          // (1 - q/2)
                    float t2 = t * t;
                    float W = kernelNorm * t2 * t2 * (1.0f + 2.0f * q);
                    densAccum += bodies.mass[j] * W;
                }
            }
            nInd = node.next;
        } else {
            nInd = node.firstChild;  // descend
        }
    }

    bodies.dens[bInd] = densAccum;
}


void quadTreeSim::computeAccelSubtree(int nInd) {
    int cnt = tree[nInd].hi - tree[nInd].lo;
    if (cnt <= ACCEL_TASK_SIZE || tree[nInd].firstChild == -1) {
        for (int i = tree[nInd].lo; i < tree[nInd].hi; i++)
            computeAccel(i, theta, bodies.axNew, bodies.ayNew, false, nullptr);
        return;
    }
    int fc = tree[nInd].firstChild;
    for (int c = 0; c < 4; c++) {
        int child = fc + c;
        if (tree[child].hi > tree[child].lo) {
            #pragma omp task firstprivate(child)
            computeAccelSubtree(child);
        }
    }
    #pragma omp taskwait
}


void quadTreeSim::computeAccel(int bInd, float thetaIn, vector<float>& axOut, vector<float>& ayOut, bool DO_INFO, double* potEnergy) {
    float totAccX = 0.0f, totAccY = 0.0f;

    float thetaInSq = sq(thetaIn);
    int nInd = 0;
    while (nInd != -1) {
        Node node = tree[nInd];

        float rX = node.comx - bodies.px[bInd];
        float rY = node.comy - bodies.py[bInd];
        float distSq = sq(rX) + sq(rY);
        float s = node.size;

        if (sq(s) < thetaInSq * distSq) {
            float distSqSoft = distSq + gravEpsilonSq;
            float invDist = 1.0f / sqrt(distSqSoft);
            float accelMag = G * node.mass * sq(invDist) * invDist;
            totAccX += accelMag * rX;
            totAccY += accelMag * rY;
            // if (DO_INFO && potEnergy) *potEnergy += -0.5 * G * node.mass * bodies.mass[bInd] * invDist;
            nInd = node.next;

        } else if (node.firstChild == -1) {
            for (int j = node.lo; j < node.hi; j++) {
                float dX = bodies.px[j] - bodies.px[bInd];
                float dY = bodies.py[j] - bodies.py[bInd];
                float dSq = sq(dX) + sq(dY) + gravEpsilonSq;
                float invDist = 1.0f / sqrt(dSq);
                float accelMag = G * bodies.mass[j] * sq(invDist) * invDist;
                totAccX += accelMag * dX;
                totAccY += accelMag * dY;
                // if (DO_INFO && potEnergy)
                    // *potEnergy += -0.5 * G * bodies.mass[j] * bodies.mass[bInd] * invDist;
            }
            nInd = node.next;
        } else {
            nInd = node.firstChild;
        }
    }
    axOut[bInd] = totAccX;
    ayOut[bInd] = totAccY;
}


void quadTreeSim::buildTree() {
    float minX = std::numeric_limits<float>::max(), minY = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest(), maxY = std::numeric_limits<float>::lowest();

    static std::vector<uint8_t> keep;
    keep.resize(N);
    int totalKeep = 0;

    // mark for culling
    float cx = viewW / 2.0f, cy = viewH / 2.0f;
    float maxR2 = sq(ESCAPE_CULL_RAD * std::min(viewW, viewH));
    #pragma omp parallel for reduction(min:minX,minY) reduction(max:maxX,maxY) reduction(+:totalKeep)
    for (int i = 0; i < N; i++) {
        float px = bodies.px[i], py = bodies.py[i];
        float dx = px - cx, dy = py - cy;
        bool k = (sq(dx) + sq(dy) < maxR2) || (bodies.mass[i] > 1.0f);
        keep[i] = k;
        totalKeep += k;
        if (k) {
            minX = std::min(minX, px);
            maxX = std::max(maxX, px);
            minY = std::min(minY, py);
            maxY = std::max(maxY, py);
        }
    }
    // do culling
    if (totalKeep < N) compactArrays(keep, totalKeep);

    std::iota(buildIndices.begin(), buildIndices.end(), 0);
    treeCold[0].cx = (maxX + minX) / 2.0f;
    treeCold[0].cy = (maxY + minY) / 2.0f;
    tree[0].size = max(abs(maxX - minX), abs(maxY - minY));
    tree[0].lo = 0;
    tree[0].hi = N;
    tree[0].next = -1;

    nodeCnt.store(1, std::memory_order_relaxed);

    #pragma omp parallel
    {
        #pragma omp single nowait
        buildSubtree(0);
    }
    reorderBodies();

    bodyToLeaf.resize(N);
    for (int n = 0; n < nodeCnt; n++) {
        if (tree[n].firstChild == -1) {
            for (int i = tree[n].lo; i < tree[n].hi; i++) {
                bodyToLeaf[i] = n;
            }
        }
    }

    computeMassDistribution();
}


void quadTreeSim::compactArrays(const std::vector<uint8_t>& keep, int totalKeep) {
    static std::vector<int> newIdx;
    newIdx.resize(N);
    int running = 0;
    for (int i = 0; i < N; i++) {
        newIdx[i] = running;
        running += keep[i];
    }

    auto compact = [&](auto& arr) {
        using T = typename std::remove_reference_t<decltype(arr)>::value_type;
        static std::vector<T> tmp;   // one per type instantiation, shared across threads
        tmp.resize(arr.size());
        #pragma omp parallel for
        for (int i = 0; i < N; i++) {
            if (keep[i]) tmp[newIdx[i]] = arr[i];
        }
        std::swap(arr, tmp);
    };

    std::apply([&](auto*... arrPtrs) {
        (compact(*arrPtrs), ...);
    }, bodies.arrays());

    N = totalKeep;
    bodies.N = totalKeep;
}


void quadTreeSim::buildSubtree(int nInd) {
    int cnt = tree[nInd].hi - tree[nInd].lo;
    if (cnt <= LEAF_CAPACITY || tree[nInd].size < MIN_NODE_SIZE) { makeLeaf(nInd); return; }

    partitionNode(nInd);
    int fc = tree[nInd].firstChild;

    if (cnt >= BUILD_TASK_SIZE) {
        // Big subtree, do tasks
        for (int c = 0; c < 4; c++) {
            if (tree[fc+c].hi > tree[fc+c].lo) {
                int child = fc + c;
                #pragma omp task firstprivate(child)
                buildSubtree(child);
            }
        }
        #pragma omp taskwait
    } else {
        // Small subtree, task overhead not worth
        for (int c = 0; c < 4; c++) {
            if (tree[fc+c].hi > tree[fc+c].lo)
                buildSubtree(fc + c);
        }
    }
}


void quadTreeSim::makeLeaf(int nInd) {
    int lo = tree[nInd].lo;
    int hi = tree[nInd].hi;
    float m = 0, wx = 0, wy = 0;
    for (int ii = lo; ii < hi; ii++) {
        int i = buildIndices[ii];
        m += bodies.mass[i];
        wx += bodies.mass[i] * bodies.px[i];
        wy += bodies.mass[i] * bodies.py[i];
    }
    tree[nInd].mass = m;
    tree[nInd].comx = m > 0 ? wx / m : 0;
    tree[nInd].comy = m > 0 ? wy / m : 0;
}


void quadTreeSim::partitionNode(int nInd) {
    float cx = treeCold[nInd].cx;
    float cy = treeCold[nInd].cy;
    int lo = tree[nInd].lo;
    int hi = tree[nInd].hi;
    int* idx = buildIndices.data();

    auto midIt = std::partition(idx + lo, idx + hi,  [&](int i) { return bodies.py[i] < cy; });
    int mid = midIt - idx;
    auto q01It = std::partition(idx + lo, idx + mid, [&](int i) { return bodies.px[i] < cx; });
    auto q23It = std::partition(idx + mid, idx + hi, [&](int i) { return bodies.px[i] < cx; });
    int splits[5] = { lo, (int)(q01It - idx), mid, (int)(q23It - idx), hi };

    subdivide(nInd);
    int fc = tree[nInd].firstChild;
    for (int c = 0; c < 4; c++) {
        tree[fc + c].lo = splits[c];
        tree[fc + c].hi = splits[c + 1];
    }
}


void quadTreeSim::reorderBodies() {
    auto reorderArray = [&](auto& arr) {
        using T = typename std::remove_reference_t<decltype(arr)>::value_type;
        static std::vector<T> tmp;     // ← was thread_local
        tmp.resize(N);
        #pragma omp parallel for
        for (int i = 0; i < N; i++) tmp[i] = arr[buildIndices[i]];
        std::swap(arr, tmp);
    };

    std::apply([&](auto*... arrPtrs) {
        (reorderArray(*arrPtrs), ...);
    }, bodies.arrays());

    std::iota(buildIndices.begin(), buildIndices.end(), 0);
}


void quadTreeSim::subdivide(int nInd) {
    int base = nodeCnt.fetch_add(4, std::memory_order_relaxed);
    tree[nInd].firstChild = base;
    for (int i = 0; i < 4; i++) {
        tree[base+i] = Node();
        tree[base+i].size = tree[nInd].size / 2;
        tree[base+i].next = (i < 3) ? base + i + 1 : tree[nInd].next;
        treeCold[base+i].cx = treeCold[nInd].cx + (tree[nInd].size/4) * ((i & 1) ? 1 : -1);
        treeCold[base+i].cy = treeCold[nInd].cy + (tree[nInd].size/4) * ((i & 2) ? 1 : -1);
    }
}


void quadTreeSim::computeMassDistribution() {
    for (int i = nodeCnt - 1; i >= 0; i--) {
        if (tree[i].firstChild == -1) continue;

        float totMass = 0, weightSumX = 0, weightSumY = 0;
        for (int c = 0; c < 4; c++) {
            int child = tree[i].firstChild + c;
            totMass += tree[child].mass;
            weightSumX += tree[child].comx * tree[child].mass;
            weightSumY += tree[child].comy * tree[child].mass;
        }
        tree[i].mass = totMass;
        tree[i].comx = totMass > 0 ? weightSumX / totMass : 0;
        tree[i].comy = totMass > 0 ? weightSumY / totMass : 0;
    }
}






quadTreeSim::quadTreeSim(int N_, double mass, double size, double viewW_, double viewH_) : N(N_), viewW(viewW_), viewH(viewH_), bodies(N_) {
    // Initialize 
    nodeCnt = 0;
    bodyCost.assign(N, 1.0f);
    tree.resize(N * 8);
    treeCold.resize(N * 8);
    buildIndices.resize(N);
    std::iota(buildIndices.begin(), buildIndices.end(), 0);


    // Base body fill
    for(int i = 0; i < N; i++) {
        bodies.mass[i] = mass;
        bodies.size[i] = size;
        bodies.energy[i] = kB * INIT_TEMP / ((adiaGamma - 1) * mass);
        bodies.diffuse[i] = 1;
    }

    // Disk settings
    double screenFracScale = 2.5;
    double diskScale = min(viewW, viewH)/screenFracScale;
    randDisk(bodies, diskScale, 0.3);

    // Create massive central body
    bodies.mass[0] = 20;
    bodies.size[0] = 10 * size;
    bodies.px[0] = 0.0f;
    bodies.py[0] = 0.0f;
    bodies.diffuse[0] = 0;

    for (int i = 1; i < 10; i++) {
        bodies.mass[i] = 0.1;
        bodies.size[i] = 5 * size;
        bodies.diffuse[i] = 0;
    }

    // Initialize into stable orbit
    buildTree();
    for(int i = 0; i < N; i++) {
        computeAccel(i, init_theta, bodies.ax, bodies.ay, false, nullptr);
    }
    setOrbitalVel(bodies, 0, 0);

    // Center to avoid drift
    double totMass = 0;
    Eigen::Vector4d com(0.0, 0.0, 0.0, 0.0);
    Eigen::Vector2f cent(viewW/2, viewH/2);
    for (int i = 0; i < N; i++) {
        com[0] += bodies.px[i] * bodies.mass[i];
        com[1] += bodies.py[i] * bodies.mass[i];
        com[2] += bodies.vx[i] * bodies.mass[i];
        com[3] += bodies.vy[i] * bodies.mass[i];
        totMass += bodies.mass[i];
    }
    com /= totMass;
    for (int i = 0; i < N; i++) {
        bodies.px[i] += -com[0] + cent[0];
        bodies.py[i] += -com[1] + cent[1];
        bodies.vx[i] += -com[2];
        bodies.vy[i] += -com[3];
    }

    // Remove central body velocity
    int centralIdx = 0;
    for (int i = 0; i < N; i++)
        if (bodies.mass[i] > 10.0f) { centralIdx = i; break; }
    bodies.vx[centralIdx] = 0; bodies.vy[centralIdx] = 0;
}



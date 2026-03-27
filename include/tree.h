#pragma once
#include <vector>
#include <Eigen/Dense>

#include "bodies.h"
#include "node.h"

class quadTreeSim {
    public:
        quadTreeSim(int N, double mass, double size, double viewW, double viewH);

        void buildTree();

        void reorderBodies();

        void step(double dt, bool DO_INFO);

        int getQuadrant(int bInd, int nInd);

        void insertParticle(int nInd);

        void subdivide(int nInd);

        void computeMassDistribution();

        void computeAccel(int bInd, float thetaIn, std::vector<float>& axOut, std::vector<float>& ayOut, bool DO_INFO, double* potEnergy);

        Bodies& getBodies() { return bodies; }

        const int getN() const { return N; }

        double viewW, viewH;
        int N;
        int nodeCnt;

    private:
        Bodies bodies;
        // std::vector<int> parents;
        std::vector<Node> tree;
        std::vector<int> order;
        std::vector<uint32_t> mortonCodes;
};

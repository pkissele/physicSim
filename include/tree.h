#pragma once
#include <atomic>
#include <vector>

#include "bodies.h"
#include "node.h"

class quadTreeSim {
    public:
        quadTreeSim(int N, double mass, double size, double viewW, double viewH);

        void step(double dt, bool LOG_ENERGY, bool LOG_TIME);

        void findDensity(int bInd);

        void queryRange(int queryIdx, float radius, std::vector<int>& results);

        void computeAccelSubtree(int nInd);

        void computeAccel(int bInd, float thetaIn, std::vector<float>& axOut, std::vector<float>& ayOut, bool DO_INFO, double* potEnergy);

        void buildTree();

        void buildSubtree(int nInd);

        void partitionNode(int nInd);

        void cullEscape(float maxRadius);

        void compactArrays(const std::vector<uint8_t>& keep, int totalKeep);

        void makeLeaf(int nInd);

        void reorderBodies();

        void subdivide(int nInd);

        void computeMassDistribution();


        Bodies& getBodies() { return bodies; }

        const int getN() const { return N; }

        double viewW, viewH;
        int N;
        // int nodeCnt;
        std::atomic<int> nodeCnt;

    private:
        Bodies bodies;

        std::vector<Node> tree;
        std::vector<NodeCold> treeCold;

        std::vector<int> buildIndices;

        std::vector<float> bodyCost;
        std::vector<int> bodyToLeaf;
};

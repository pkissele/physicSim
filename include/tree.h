#pragma once
#include <vector>
#include <Eigen/Dense>

#include "body.h"
#include "node.h"

using Vec2 = Eigen::Vector2d;

class quadTreeSim {
    public:
        quadTreeSim(int N, double mass, double size, double viewW, double viewH);

        void buildTree();

        void step(double dt, bool DO_INFO);

        int getQuadrant(int bInd, int nInd);

        void insertParticle(int nInd);

        void subdivide(int nInd);

        void computeMassDistribution();

        Vec2 computeAccel(int bInd, double thetaIn, bool DO_INFO, double* potEnergy);


        const std::vector<Body>& getBodies() const { return bodies; }

        const int getAlive() const {return aliveN; }

        double viewW, viewH;
        int N;
        int aliveN;
        int nodeCnt;

    private:
        std::vector<Body> bodies;
        std::vector<int> parents;
        std::vector<Vec2> accel;
        std::vector<Vec2> accelNew;
        std::vector<Node> tree;
};

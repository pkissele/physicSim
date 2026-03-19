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

        void insertParticle(int nInd, int bInd);

        void subdivide(int nInd);

        void computeMassDistribution(int nInd);

        Vec2 computeAccel(int bInd, int nInd, double* potEnergy);

        void collide(int b1, int b2);

        void deleteMarkedBodies();

        void doCollisions();

        std::vector<Vec2> computeAccelerations(bool DO_INFO, double* potEnergy);
 
        void step(double dt, bool DO_INFO);

        const std::vector<Body>& getBodies() const { return bodies; }

        const int getAlive() const {return aliveN; }

        double viewW, viewH;
        int N;
        int aliveN;
        int nodeCnt;

    private:
        std::vector<Body> bodies;
        std::vector<int> markDelete;
        std::vector<Vec2> accel;
        std::vector<Node> tree;
};

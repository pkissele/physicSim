#pragma once
#include <vector>
#include <Eigen/Dense>

#include "body.h"

using Vec2 = Eigen::Vector2d;

class NBodySimulation {
    public:
        NBodySimulation(int N, double mass, double viewW, double viewH);

        std::vector<Vec2> computeAccelerations(bool DO_INFO, double &potEnergy);
 
        void step(double dt, bool DO_INFO);
        const std::vector<Body>& getBodies() const { return bodies; }

        double viewW, viewH;
        int N;

    private:
        std::vector<Body> bodies;
        std::vector<Vec2> accel;
};

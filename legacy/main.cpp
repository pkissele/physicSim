#include <fstream>
#include <iostream>
#include <tuple>
#include <array>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <numeric>
#include <sstream>
#include <time.h>
#include <stdlib.h>
#include <random>
#include <filesystem>
#include <cstdlib>
#include <iomanip>


#include <Eigen/Dense>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_odeiv2.h>
#include <cairo.h>

using namespace std;

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec2 = Eigen::Vector2d;


struct Body {
    double mass;
    Vec2 pos;
    Vec2 vel;
};


std::mt19937 rng(std::random_device{}());  // random seed
// std::mt19937 rng(0);  // fixed seed
std::uniform_real_distribution<double> randDist(0.0, 1.0);


const double G = 1; // Arbitrary for now
const double pi = 3.14159265;

// Display output dimensions
const int screenH = 900;
const int screenW = 900;

/// Object space "view" dimensions -> [0, 10] x [0, 10]
const int viewH = 10;
const int viewW = 10;


void draw_frames(const vector<vector<Vec2>>& positions, const std::string& folder, int frames) {
    cairo_surface_t* surface = cairo_image_surface_create(CAIRO_FORMAT_ARGB32, screenW, screenH);
    cairo_t* cr = cairo_create(surface);

    // background
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_paint(cr);

    int frameSkip = (int)positions.size()/frames;
    for(int i = 0; i < frames; i++) {
        // cout << "Frame " << i << endl;
        cairo_set_source_rgb(cr, 0, 0, 0);
        cairo_paint(cr);

        int cFrame = i*frameSkip;

        // draw particles
        cairo_set_source_rgb(cr, 1,1,1);
        for (auto& p : positions[cFrame]) {
            double px = (p[0])/(viewW) * screenW;  // map [0,1] -> [0,width]
            double py = screenH-((p[1])/(viewH) * screenH); // map [0,1] -> [0,height]
            cairo_arc(cr, px, py, 3, 0, 2*pi);
            cairo_fill(cr);
        }

        string filename = folder + "/frame_" + to_string(i) + ".png";

        cairo_surface_write_to_png(surface, filename.c_str());
    }
    cairo_destroy(cr);
    cairo_surface_destroy(surface);
}


int main() {
    const int N = 100;
    vector<Body> bodies(N);


    for(int b = 0; b < N; b++) {
        bodies[b] = Body{0.5, Vec2(randDist(rng)*viewW, randDist(rng)*viewH), Vec2(0, 0)};
    }

    vector<vector<double>> extForces(1);
    extForces[0] = {0, 0};

    const bool BORDER_COLLIDE = false;

    double collisionRad = 0.1;

    double t = 0;
    double endT = 10;
    double dt = 1e-5;
    int timeSteps = (endT-t)/dt;

    const double gravEpsilon = 0.05;
    const double gravEpsilon2 = pow(gravEpsilon, 2);


    cout << "Simulating with " << timeSteps << " steps and a time step of " << dt << endl;

    vector<vector<Vec2>> positions(timeSteps, vector<Vec2>(N));

    for (int frame=0; frame < timeSteps; frame++) {
        bool INFO_FLAG = false;
        double potEnergy = 0;
        double kinEnergy = 0;

        if(frame%((int)timeSteps/10) == 0) {
            INFO_FLAG = true;
            cout << (int)(100 * frame/timeSteps) << "\% complete" << endl;
        }
        double ti = t + dt;
        // update particles positions
        for (int i = 0; i < N; i++) {
            Vec2 accel = {0, 0};

            for (int f = 0; f < extForces.size(); f++) {
                accel[0] += extForces[f][0]*dt;
                accel[1] += extForces[f][1]*dt;
            }

            bool collide = false;
            // Gravity between bodies
            for (int j = 0; j < N; j++) {
                if(j == i) continue;
                Vec2 dir = {bodies[j].pos[0] - bodies[i].pos[0], bodies[j].pos[1] - bodies[i].pos[1]};
                double distSq = pow(bodies[i].pos[0]-bodies[j].pos[0], 2)+pow(bodies[i].pos[1]-bodies[j].pos[1], 2);
                // if(distSq <= pow(collisionRad, 2)) {
                //     collide = true;
                // }
                // Softened gravity: G * m1 * m2 * r/((r^2+eps^2)^3/2)
                double gravMag = G * bodies[j].mass * bodies[i].mass * (sqrt(distSq)) / pow(distSq + gravEpsilon2, 1.5);
                if(INFO_FLAG) {
                    potEnergy += -1 * G * bodies[j].mass * bodies[i].mass / pow(distSq + gravEpsilon2, 0.5);
                }
                // double gravMag = G * bodies[j].mass * bodies[i].mass / distSq;
                accel[0] += (dir[0]/sqrt(distSq)) * gravMag;
                accel[1] += (dir[1]/sqrt(distSq)) * gravMag;
            }

            bodies[i].vel[0] += accel[0]*dt;
            bodies[i].vel[1] += accel[1]*dt;

            if(collide) {
                bodies[i].vel[0] *= -1;
                bodies[i].vel[1] *= -1;
            }

            bodies[i].pos[0] += bodies[i].vel[0]*dt;
            bodies[i].pos[1] += bodies[i].vel[1]*dt;

            if(info_flag) {
                kinenergy += 1/2 * bodies[i].mass * sqrt(pow(bodies[i].vel[0], 2) + pow(bodies[i].vel[1], 2));
            }

            if(BORDER_COLLIDE) {
                if((bodies[i].pos[1] < 0) && (bodies[i].vel[1] < 0)) {
                    bodies[i].vel[1] *= -1;
                }
                else if((bodies[i].pos[1] > viewH) && (bodies[i].vel[1] > 0)) {
                    bodies[i].vel[1] *= -1;
                }
                else if((bodies[i].pos[0] < 0) && (bodies[i].vel[0] < 0)) {
                    bodies[i].vel[0] *= -1;
                }
                else if((bodies[i].pos[0] > viewW) && (bodies[i].vel[0] > 0)) {
                    bodies[i].vel[0] *= -1;
                }
            }
            positions[frame][i] = bodies[i].pos;
        }

        if(INFO_FLAG) {
            cout << "Energy: " ;
            cout << scientific << kinEnergy + potEnergy << endl;
        }
    }

    cout << "Simulation complete, begin drawing" << endl;

    int FPS = 30; // frame display speed
    double playback = 1; //inverse
    int nFrames = endT * FPS * playback;

    draw_frames(positions, "output/", nFrames);

    string pyComm = "python img2gif.py output " + to_string(nFrames) + " " + to_string((int)(1000/FPS));
    system(pyComm.c_str());

    return 0;
}

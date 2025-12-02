#include <bits/stdc++.h>
#include <iostream>
#include <string>
using namespace std;

const double pi = 3.14159;
const char newline = '\n';

int main()
{
    // cout << "Hello world!" << endl;
    double v_x = 1, v_y = 1;
    double t = 0, dt = 0.0000001, g = -9.81;
    double x;
    decltype(g) y; // decltype(g) initializes y with the same type as g
    double period = 0, prev_t = 0; // idk why underscored variables look weird...

    x = y = 0;
    bool moving_up = true;
    double v_amplitude = v_y;
    float COR = 0.9; // COR is coefficient of restitution which measures the 
    // kinetic energy conserved in a collision between two bodies
    // expressed as ratio between relative velocity after to relative vecocity before

    vector<double> results = {0.0, 1.0, 2.0, 3.0};
    // results.pushback({4, 5});
    cout << results[0] << ", " << results[1] << endl;
    cout << results[2] << ", " << results[3] << endl;

    // cout << results[2] << endl;
    v_amplitude = 0;

    while (v_amplitude > 0.5){
        t += dt;
        v_y += g*dt;
        y += v_y*dt;
        x += v_x*dt;
        if (v_y < 0 && moving_up){
            cout << "t: " << t << endl;
            cout << "x: " << x << endl;
            cout << "y: " << y << " at max height" << endl;
            cout << newline;

            moving_up = false;
        }
        else if (y < 0 && !moving_up){
            // bounce:
            period = t - prev_t;
            prev_t = t;
            v_y = -COR*v_y;
            v_amplitude = v_y;

            cout << "t: " << t << endl;
            cout << "x: " << x << endl;
            cout << "period: " << period << endl;
            cout << "v_y: " << v_y << endl;
            cout << newline;

            moving_up = true;
        }

        // cout << x << " " << y << endl;
        // cout << newline;
    }
}
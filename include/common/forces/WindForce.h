#pragma once

#include <GL/glut.h>

#include "Force.h"
#include "GlobalVars.h"

#define inv_rand_max 1 / RAND_MAX
#define e 2.71828182845904523536028747135266
#define PI 3.1415926535897932384626433832795

// https://stackoverflow.com/questions/8798771/perlin-noise-for-1d

class WindForce : public Force {
    bool *blow;
    double *dt;
    double force;

public:
    WindForce(bool *_blow, double *_dt) : blow(_blow), dt(_dt) {};

    virtual void calculate_forces(GlobalVars *globals) {
        if (*blow) {
            // non-periodic positive sine
            // 0.05 * (-sin(0.5 * x) - sin(-2 * e * x) + sin(-pi * x)) + 0.1
            force = 0.05 * (-sin(0.5 * (*dt)) - sin(-2 * e * (*dt)) + sin(-PI * (*dt))) + 0.15;
            for (int i: iVector) {
                globals->Q[2 * i] += force;
            }
        }
    }

    virtual void draw() {
        if (*blow) {
            glBegin(GL_LINES);
            glColor3f(0.1, 0.8, 1.0);
            glVertex2f(0, 0);
            glColor3f(0.1, 0.8, 1.0);
            glVertex2f(force, 0);
            glEnd();
        }
    }
};
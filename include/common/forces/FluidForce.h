#pragma once

#include "Force.h"
#include "GlobalVars.h"

#include <gfx/vec2.h>

#define PI 3.1415926535897932384626433832795
#define IX(i,j) ((i)+(N_f+2)*(j))

class FluidForce : public Force {
public:
    int N_f;
    float *u;
    float *dens;
    float *v;

    FluidForce(int _N_f, float *_u, float *_v, float *_dens): N_f(_N_f), u(_u), v(_v), dens(_dens){}

    virtual void calculate_forces(GlobalVars *globals) {
        for (int i: iVector) {
            Vec2 par_pos = globals->get_pos(i);
            Vec2 par_vel = globals->get_vel(i);

           int q = (par_pos[0] + 1) / 2;
           int r = (par_pos[1] + 1) / 2;

            q = q * N_f + 1;
            r = r * N_f + 1;


            float fluid_force_x = 1.0/2.0 * dens[IX(q,r)] * (u[IX(q,r)]);
            float fluid_force_y = 1.0/2.0 * dens[IX(q,r)] * (v[IX(q,r)]);
            
            globals->Q[2 * i] += fluid_force_x * 10;
            globals->Q[2 * i + 1] += fluid_force_y * 10;
        }
    }
};

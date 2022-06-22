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
    float *v;

    FluidForce(int _N_f, float *_u, float *_v): N_f(_N_f), u(_u), v(_v){}

    virtual void calculate_forces(GlobalVars *globals) {
        for (int i: iVector) {
//            int  k = x[2 * i]
//            int  j = x[2 * i + 1]
//            globals->Q[2 * i] += m_constant[0] / globals->W[2 * i];
//            globals->Q[2 * i + 1] += m_constant[1] / globals->W[2 * i + 1];

            //fluid
//            i = (int)((       mx /(float)win_x)*N_f+1);
//            j = (int)(((win_y-my)/(float)win_y)*N_f+1);
//
//            //particle coord
//            float q = mx / (float) win_x;
//            float r = (win_y - my) / (float) win_y;
//            q = q * 2 - 1;
//            r = r * 2 - 1;
//            mouse_particle->m_Position = Vec2(q, r);


            Vec2 par_pos = globals->get_pos(i);
            Vec2 par_vel = globals->get_vel(i);

            int q = par_pos[0] * N_f + 1;
            int r = par_pos[1] * N_f + 1;

//            int q = (par_pos[0] + 1) / 2;
//            int r = (par_pos[1] + 1) / 2;
//
//            q = q * N_f + 1;
//            r = r * N_f + 1;

            // Fi = -6 * pi * Fluid viscosity * particle radius * ( velocity of particle - average velocity of cell)
//            float fluid_force_x = -6 * PI * (par_vel[0] - u[IX(q,r)]);
//            float fluid_force_y = -6 * PI * (par_vel[1] - v[IX(q,r)]);

            //Second approach
//            float fluid_force_x = 1/2 * dens[IX(q,r)] * (u[IX(q,r)] * u[IX(q,r)] + v[IX(q,r)] * v[IX(q,r)]);
//            float fluid_force_y = 1/2 * dens[IX(q,r)] * (u[IX(q,r)] * u[IX(q,r)] + v[IX(q,r)] * v[IX(q,r)]);

//            std::cout << "fluid_force_x: "<< fluid_force_x << std::endl;


//            globals->Q[2 * i] += fluid_force_x;
//            globals->Q[2 * i + 1] += fluid_force_y;



            // Approach 2
            //f = q * n
            //q = 1/2 * rho * uTu
        }
    }
};

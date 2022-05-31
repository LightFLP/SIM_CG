#pragma once

#include <gfx/vec2.h>

#include "Force.h"

class WallRepulsionForce : public Force {
    double m_lBoundary, m_rBoundary;
    bool *collision;
    double damping = 0.7;
    double epsilon = 0.0075;
public:
    WallRepulsionForce(bool *_collision, double lBoundary, double rBoundary) : collision(_collision),
    m_lBoundary(lBoundary), m_rBoundary(rBoundary) {}

    virtual void calculate_forces(GlobalVars *globals) {
        if(*collision)
        {
            for (int i: iVector) {
                Vec2 p0 = globals->get_pos(i);

                if(p0[0] >= m_rBoundary-epsilon){
                    globals->x[2 * i] = m_rBoundary - 2*epsilon;
                    globals->v[2 * i] = -damping*(abs(globals->v[2 * i]));
                    globals->Q[2 * i] = 0;
                    //globals->Q[2 * i + 1] += m_constant[1] / globals->W[2 * i + 1];
                }else if(p0[0] <= m_lBoundary + epsilon){
                    globals->x[2 * i] = m_lBoundary + 2*epsilon;
                    globals->v[2 * i] = damping*(abs(globals->v[2 * i]));
                    globals->Q[2 * i] = 0;
                    //globals->Q[2 * i + 1] += m_constant[1] / globals->W[2 * i + 1];
                }
            }
        }
    }
};

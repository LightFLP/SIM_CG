#pragma once

#include <Forces/Force.h>

class ConstantForce : public Force{
    public:
        Vec2 m_constant;
        ConstantForce(Vec2 acceleration){
            m_constant = acceleration;
        }
        
        virtual void calculate_forces(GlobalVars* globals){
            for (int i : iVector){
                globals->Q[2*i] += m_constant[0]/globals->W[2*i];
                globals->Q[2*i+1] += m_constant[1]/globals->W[2*i+1];
            }
        }
};

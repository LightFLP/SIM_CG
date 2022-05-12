#pragma once

#include <Force.h>

class ConstantForce : public Force{
    public:
        Vec2 m_constant;
        ConstantForce(Vec2 acceleration){
            m_constant = acceleration;
        }
        
        virtual void calculate_forces(){
            for (Particle* p : pVector){
                p->m_ForceAccum += m_constant*p->m_Mass;
            }
        }
};

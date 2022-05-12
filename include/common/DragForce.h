#pragma once

#include <Force.h>

class DragForce : public Force{
    public:
        double m_kd;
        DragForce(double kd){
            m_kd = kd;
        }
        
        virtual void calculate_forces(){
            for (Particle* p : pVector){
                p->m_ForceAccum -= m_kd * p->m_Velocity;
            }
        }
};
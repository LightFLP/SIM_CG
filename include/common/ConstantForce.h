#include <Force.h>

class ConstantForce : public Force{
    public:
        Vec2f m_constant;
        ConstantForce(Vec2f acceleration){
            m_constant = acceleration;
        }
        
        virtual void calculate_forces(){
            for (Particle* p : pVector){
                p->m_ForceAccum += m_constant;
            }
        }
};
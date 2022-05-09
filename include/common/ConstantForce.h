#include <Force.h>

class ConstantForce : public Force{
    public:
        Vec2f m_constant;
        ConstantForce(Vec2f acceleration){
            m_constant = acceleration;
        }
        std::vector<Vec2f> m_forces = std::vector<Vec2f>(0);
        virtual Vec2f get_force_at(int index){
            return m_forces[index];
        }

        virtual void calculate_forces(std::vector<Particle*> pVector, float dt){
            if (m_forces.size() != pVector.size()){
                m_forces = std::vector<Vec2f>(pVector.size());
                for (int i = 0; i < pVector.size(); ++i){
                    m_forces[i] = pVector[i]->m_Mass * m_constant;
                }
            }
        }
};
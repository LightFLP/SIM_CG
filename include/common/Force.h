#include <Particle.h>
#include <gfx/vec2.h>
#include <vector>


class Force{
    public:
        virtual Vec2f get_force_at(int index);
        virtual void calculate_forces(std::vector<Particle*> pVector, float dt){}
};
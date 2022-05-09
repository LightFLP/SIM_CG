#include <Particle.h>
#include <vector>

class Constraint{
    protected:
        std::vector<Particle*> pVector = std::vector<Particle*>();
    public:
        void register_particle(Particle* p){
            pVector.push_back(p);
        }
        virtual void calculate_forces(){};
        virtual void draw(){};
};
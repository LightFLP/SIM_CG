#pragma once

#include <Particle.h>
#include <gfx/vec2.h>
#include <vector>
#include <GlobalVars.h>


class Force{
    protected:
        std::vector<int> iVector = std::vector<int>();
    public:
        void register_particle(int p_index){
            iVector.push_back(p_index);
        }
        virtual void calculate_forces(GlobalVars* globals){}
        virtual void draw(){}
};
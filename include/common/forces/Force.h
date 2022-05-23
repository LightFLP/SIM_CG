#pragma once

#include <vector>

class GlobalVars;

class Force {
protected:
    std::vector<int> iVector = std::vector<int>();

public:
    void register_particle(int p_index) {
        iVector.push_back(p_index);
    }

    virtual void calculate_forces(GlobalVars *globals) {}

    virtual void draw() {}

    static std::vector<Force *> _forces;
    static std::vector<Force *> _mouse_forces;

    static void AddForce(Force *force);
    static void AddMouseForce(Force *mouse_force);
};
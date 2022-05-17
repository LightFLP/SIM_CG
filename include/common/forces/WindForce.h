#pragma once

#include <Forces/Force.h>

class WindForce : public Force {
    bool *blow;
    double const strength;

public:
    WindForce(bool *_blow, double const _strength) : blow(_blow), strength(_strength) {};

    virtual void calculate_forces(GlobalVars* globals) {
        for (int i : iVector) {
            globals->Q[2*i] += strength * (1 - globals->x[2*i]) * (*blow);
        }
    }
};
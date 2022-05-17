#pragma once

#include <Forces/Force.h>

#define inv_rand_max 1 / RAND_MAX

class WindForce : public Force {
    bool *blow;

public:
    WindForce(bool *_blow) : blow(_blow) {};

    virtual void calculate_forces(GlobalVars* globals) {
        if (*blow) {
            for (int i : iVector) {
                globals->Q[2*i] += 0.5 * rand() * inv_rand_max * (1 - globals->x[2*i]);
            }
        }
    }
};
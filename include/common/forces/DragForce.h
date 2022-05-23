#pragma once

#include "Force.h"
#include "GlobalVars.h"

class DragForce : public Force {
public:
    double m_kd;

    DragForce(double kd) {
        m_kd = kd;
    }

    virtual void calculate_forces(GlobalVars *globals) {
        for (int i: iVector) {
            globals->Q[2 * i] -= m_kd * globals->v[2 * i];
            globals->Q[2 * i + 1] -= m_kd * globals->v[2 * i + 1];
        }
    }
};
#pragma once

#include "Particle.h"
#include "Constraint.h"

class StaticConstraint : public Constraint {
public:
    StaticConstraint(Particle* p, int p_index, int c_index);

    virtual void draw();

    virtual double eval_C();
    virtual double eval_Cdot();
    virtual void eval_J();
    virtual void eval_Jdot();
private:
    Vec2 const m_StaticPos;
};
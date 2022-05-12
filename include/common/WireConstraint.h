#pragma once

#include "Particle.h"
#include "Constraint.h"

class WireConstraint : public Constraint {
public:
    WireConstraint(Particle* p, int p_index, int c_index, const double angle);

    virtual void draw();

    virtual double eval_C();
    virtual double eval_Cdot();
    virtual void eval_J();
    virtual void eval_Jdot();
private:
    Vec2 const m_Center;
    double const m_Angle;
};
#pragma once

#include "Particle.h"
#include "Constraint.h"

class WireConstraint : public Constraint {
public:
    WireConstraint(int p_index, int c_index, const double angle, Vec2 center);

    virtual void draw();

    virtual double eval_C(GlobalVars* globals);
    virtual double eval_Cdot(GlobalVars* globals);
    virtual void eval_J(GlobalVars* globals, std::vector<MatrixBlock> & blocks);
    virtual void eval_Jdot(GlobalVars* globals, std::vector<MatrixBlock> & blocks);
private:
    Vec2 const m_Center;
    double const m_Angle;
};
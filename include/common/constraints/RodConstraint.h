#pragma once

#include <gfx/vec2.h>

#include "Constraint.h"

class RodConstraint : public Constraint {
    double const m_dist;
    Vec2 p0, p1;

public:
    RodConstraint(int p_index1, int p_index2, double dist);

    virtual void draw();

    virtual double eval_C(GlobalVars *globals);
    virtual double eval_Cdot(GlobalVars *globals);
    virtual void eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks);
    virtual void eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks);
};

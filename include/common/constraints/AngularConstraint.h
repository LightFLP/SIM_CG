#pragma once

#include <gfx/vec2.h>

#include "Constraint.h"

class AngularConstraint : public Constraint {
    double const m_ra;
    Vec2 p0, p1, p2;

public:
    AngularConstraint(int p_index1, int p_index2, int p_index3, double ra);

    virtual void draw();

    virtual double eval_C(GlobalVars *globals);
    virtual double eval_Cdot(GlobalVars *globals);
    virtual void eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks);
    virtual void eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks);
};
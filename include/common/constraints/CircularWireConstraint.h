#pragma once

#include <gfx/vec2.h>

#include "Constraint.h"

class CircularWireConstraint : public Constraint {
    Vec2 const m_center;
    double const m_radius;

public:
    CircularWireConstraint(int p_index, const Vec2 center, const double radius);

    virtual void draw();

    virtual double eval_C(GlobalVars *globals);
    virtual double eval_Cdot(GlobalVars *globals);
    virtual void eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks);
    virtual void eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks);
};

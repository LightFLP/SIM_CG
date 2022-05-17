#pragma once

#include "Constraint.h"

class WireConstraint : public Constraint {
    double const m_Height;

public:
    WireConstraint(int p_index, const double m_Height);

    virtual void draw();

    virtual double eval_C(GlobalVars* globals);
    virtual double eval_Cdot(GlobalVars* globals);
    virtual void eval_J(GlobalVars* globals, std::vector<MatrixBlock> & blocks);
    virtual void eval_Jdot(GlobalVars* globals, std::vector<MatrixBlock> & blocks);
};
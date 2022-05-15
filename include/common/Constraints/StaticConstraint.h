#pragma once

#include "Particle.h"
#include "Constraint.h"

class StaticConstraint : public Constraint {
public:
    StaticConstraint(int p_index, int c_index, Vec2 static_pos);

    virtual void draw();

    virtual double eval_C(GlobalVars* globals);
    virtual double eval_Cdot(GlobalVars* globals);
    virtual void eval_J(GlobalVars* globals, std::vector<MatrixBlock> & blocks);
    virtual void eval_Jdot(GlobalVars* globals, std::vector<MatrixBlock> & blocks);
private:
    Vec2 const m_StaticPos;
};
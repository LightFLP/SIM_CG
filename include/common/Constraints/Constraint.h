#pragma once

#include <vector>

#include "linearSolver.h"
#include "GlobalVars.h"

class Constraint {
protected:
    int m_c_index;
    std::vector<int> iVector = std::vector<int>();

public:
    virtual void draw() {};

    virtual double eval_C(GlobalVars *globals) = 0;
    virtual double eval_Cdot(GlobalVars *globals) = 0;
    virtual void eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks) = 0;
    virtual void eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks) = 0;

    static std::vector<Constraint*> _constraints;
    static void addConstraint(Constraint* constraint);

};
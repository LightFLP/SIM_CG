#pragma once

#include "Solver.h"

class EulerSolver : public Solver {
public:
    virtual void simulation_step(State *state, double dt);
};

class SympleticEulerSolver : public Solver {
public:
    virtual void simulation_step(State *state, double dt);
};


#pragma once

#include "EulerSolvers.h"

class MidpointSolver : public Solver {
    State *midpoint_state = nullptr;
    Solver *solver = new EulerSolver();

public:
    MidpointSolver(Solver *_solver = nullptr);

    virtual void simulation_step(State *state, double dt);
};

class SympleticMidpointSolver : public Solver {
    State *midpoint_state = nullptr;
    Solver *solver = new SympleticEulerSolver();

public:
    SympleticMidpointSolver(Solver *_solver = nullptr);

    virtual void simulation_step(State *state, double dt);
};
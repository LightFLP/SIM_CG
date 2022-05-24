#pragma once

class State;

class Solver {
public:
    virtual void simulation_step(State *state, double dt) = 0;
};
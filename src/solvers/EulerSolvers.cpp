#include "EulerSolvers.h"
#include "Simulator.h"
#include "GlobalVars.h"
#include "util.h"

void EulerSolver::simulation_step( State* state, double dt ){
    for(int i = 0; i < 2*state->globals->n; i++){
        state->globals->x[i] += dt*state->globals->v[i];
        state->globals->v[i] += dt*state->globals->Q[i];
    }
}

void SympleticEulerSolver::simulation_step( State* state, double dt ){
    //Util::PrintGlobals(state->globals, "SEUGlobals before.");
    for(int i = 0; i < 2*state->globals->n; i++){
        state->globals->v[i] += dt*state->globals->Q[i];
        state->globals->x[i] += dt*state->globals->v[i];
    }
    //Util::PrintGlobals(state->globals, "SEUGlobals after.");
}


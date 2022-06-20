#include "EulerSolvers.h"

#include "Simulator.h"
#include "GlobalVars.h"
#include "util.h"
#include "rigid_body.h"

void EulerSolver::simulation_step(State *state, double dt) {
    for (int i = 0; i < 2 * state->globals->n; i++) {
        state->globals->x[i] += dt * state->globals->v[i];
        state->globals->v[i] += dt * state->globals->Q[i];
    }

    double *new_state = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *old_state = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *deriv = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);

    for (RigidBody *r : RigidBody::_bodies) {
        old_state = r->getState();
        deriv = r->getDerivState();

        for (int i = 0; i < RigidBody::STATE_SIZE; i++) {
            new_state[i] = old_state[i] + deriv[i] * dt;
        }
        r->setState(new_state);
    }
}

void SympleticEulerSolver::simulation_step(State *state, double dt) {
    //Util::PrintGlobals(state->globals, "SEUGlobals before.");
    for (int i = 0; i < 2 * state->globals->n; i++) {
        state->globals->v[i] += dt * state->globals->Q[i];
        state->globals->x[i] += dt * state->globals->v[i];
    }
    //Util::PrintGlobals(state->globals, "SEUGlobals after.");

    double *new_state = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *old_state = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *deriv = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);

    for (RigidBody *r : RigidBody::_bodies) {
        old_state = r->getState();
        deriv = r->getDerivState();

        for (int i = 0; i < RigidBody::STATE_SIZE; i++) {
            new_state[i] = old_state[i] + deriv[i] * dt;
        }
        r->setState(new_state);
    }
}

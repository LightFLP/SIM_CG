#include "Simulator.h" //State import
#include "RK4Solver.h"
#include "GlobalVars.h"
#define ONE_SIXTH 1/6.0

void RK4Solver::simulation_step( State* state, double dt ){
    if (!state_k2) {
        state_k2 = new State(state, solver);
        state_k3 = new State(state, solver);
        state_k4 = new State(state, solver);
    } 
    //k1 = slope at current time
    //k2 = slope at time h/2 progressed over slope k1
    std::memcpy(state_k2->globals->data, state->globals->data, state->globals->size * sizeof(double));
    state_k2->advance(dt/2);
    //k3 = slope at time h/2 progressed over slope k2
    std::memcpy(state_k3->globals->data, state->globals->data, state->globals->size * sizeof(double));
    // Note: if using semi-implicit integration (which it does by default) you don't need this memcpy
    std::memcpy(state_k3->globals->v, state_k2->globals->v, state_k2->globals->n * 2 * sizeof(double));
    std::memcpy(state_k3->globals->Q, state_k2->globals->Q, state_k2->globals->n * 2 * sizeof(double));
    state_k3->advance(dt/2);
    //k4 = slope at time h   progressed over slope k3
    std::memcpy(state_k4->globals->data, state->globals->data, state->globals->size * sizeof(double));
    // Note: if using semi-implicit integration (which it does by default) you don't need this memcpy
    std::memcpy(state_k4->globals->v, state_k3->globals->v, state_k3->globals->n * 2 * sizeof(double));
    std::memcpy(state_k4->globals->Q, state_k3->globals->Q, state_k3->globals->n * 2 * sizeof(double));
    state_k4->advance(dt);

    for(int i = 0; i < state->globals->n*2; i++){
        state->globals->x[i] += ONE_SIXTH * (state->globals->v[i] + 2*state_k2->globals->v[i] + 2*state_k3->globals->v[i] + state_k4->globals->v[i]) * dt;
        state->globals->v[i] += ONE_SIXTH * (state->globals->Q[i] + 2*state_k2->globals->Q[i] + 2*state_k3->globals->Q[i] + state_k4->globals->Q[i]) * dt;
    }

    double *new_state = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *old_state = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *deriv = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);

    double *k1 = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *k2 = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *k3 = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);
    double *k4 = (double *) malloc(sizeof(double) * RigidBody::STATE_SIZE);

    for (RigidBody *r : RigidBody::_bodies) {
        old_state = r->getState();

        // k1
        deriv = r->getDerivState();
        for (int i = 0; i < RigidBody::STATE_SIZE; i++) {
            k1[i] = old_state[i] + deriv[i] * dt * 0.5;
        }
        r->setState(new_state);

        // k2
        deriv = r->getDerivState();
        for (int i = 0; i < RigidBody::STATE_SIZE; i++) {
            k2[i] = old_state[i] + deriv[i] * dt * 0.5;
        }
        r->setState(new_state);

        // k3
        deriv = r->getDerivState();
        for (int i = 0; i < RigidBody::STATE_SIZE; i++) {
            k3[i] = old_state[i] + deriv[i] * dt;
        }
        r->setState(new_state);

        // k4
        deriv = r->getDerivState();
        for (int i = 0; i < RigidBody::STATE_SIZE; i++) {
            k4[i] = old_state[i] + deriv[i] * dt;
        }

        for (int i = 0; i < RigidBody::STATE_SIZE; i++) {
            new_state[i] = ONE_SIXTH * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        }
        r->setState(new_state);
    }
}
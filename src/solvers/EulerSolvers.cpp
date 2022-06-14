#include "EulerSolvers.h"

#include "Simulator.h"
#include "GlobalVars.h"
#include "util.h"
#include "rigid_body.h"
#include "Particle.h"

void EulerSolver::simulation_step(State *state, double dt) {
    for (int i = 0; i < state->globals->n; i++) {
//        printf("%s", RigidBody::_rigid_indices[i] ? "true" : "false");
        if (RigidBody::_rigid_indices[i]) {
//            printf("%f\n", state->globals->Q[i*2]);
//            printf("%f\n", state->globals->Q[i*2+1]);
            RigidBody::_bodies[i%RigidBody::_bodies.size()]->force += Vec2(state->globals->Q[i*2],
                                                                       state->globals->Q[i*2+1]);
        } else {
            state->globals->x[i*2] += dt * state->globals->v[i*2];
            state->globals->x[i*2+1] += dt * state->globals->v[i*2+1];
            state->globals->v[i*2] += dt * state->globals->Q[i*2];
            state->globals->v[i*2+1] += dt * state->globals->Q[i*2+1];
        }
    }
//    printf("hi2");
    for (RigidBody *r : RigidBody::_bodies) {
        for (int i = 0; i < r->indices.size() * 2; i++) {
            state->globals->x[i] += dt * (r->position[i%2] + state->globals->x[i]);
            state->globals->v[i] += dt * r->force[i%2] / r->mass;
        }
    }
}

void SympleticEulerSolver::simulation_step(State *state, double dt) {
    //Util::PrintGlobals(state->globals, "SEUGlobals before.");
    for (int i = 0; i < 2 * state->globals->n; i++) {
        if (RigidBody::_rigid_indices[i]) {
            RigidBody::_bodies[i]->force += state->globals->Q[i];
        } else {
            state->globals->v[i] += dt * state->globals->Q[i];
            state->globals->x[i] += dt * state->globals->v[i];
        }
    }
    for (RigidBody *r : RigidBody::_bodies) {
        for (int i = 0; i < r->indices.size() * 2; i++) {
            state->globals->v[i] += dt * r->force[i%2] / r->mass;
            state->globals->x[i] += dt * state->globals->v[i];
        }
    }
    //Util::PrintGlobals(state->globals, "SEUGlobals after.");
}


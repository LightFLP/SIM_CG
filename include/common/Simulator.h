#ifndef SIMULATOR_H
#define SIMULATOR_H
// #define DEBUG
#include "Solver.h"
#include "Particle.h"
#include "Constraint.h"
#include "Force.h"
#include "linearSolver.h"
#include "EulerSolvers.h"
#include "GlobalVars.h"
#include <vector>


//TODO: Advance/evaluate

class State{
    Solver* solver;
    int n, m;

    implicitMatrixWithTrans* J;
    implicitMatrixWithTrans* Jdot;

    implicitJWJt* JWJt;
    double* lambda;

    
    // setup RHS of:
    // JWJt * lambda = - Jq - JWQ - alphaC - betaCdot
	double*  Jq;
	double* JWQ;
	double* ksC;
	double* kdC;
    double* RHS;

    double alpha = 0;
    double beta  = 0;

    void setup_globals(std::vector<Particle*> &particles);

public:
    GlobalVars* globals;
    State(Solver* _solver, int _n, int _m, std::vector<Particle*> &particles);
    
    void reset(std::vector<Particle*> &particles);

    ~State(){
        free(globals);
        free(Jq);
        free(JWQ);
        free(ksC);
        free(kdC);
        free(RHS);
    }

    void advance(double dt, std::vector<Particle*> &particles,
                            std::vector<Force*> &forces, 
                            std::vector<Force*> &mouse_forces);

    GlobalVars* evaluate(double dt, Solver* eval_solver = NULL);

    void copy_to_particles(std::vector<Particle*> &particles);
};

#endif
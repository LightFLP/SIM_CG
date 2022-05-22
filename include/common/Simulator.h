#pragma once
// #define DEBUG
#include <vector>
#include <cstdlib>

class Solver;
class implicitMatrixWithTrans;
class implicitJWJt;
class Particle;
class GlobalVars;

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

    void setup_calc_mem();

    State(State* other, Solver* solver = nullptr);

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

    void advance(double dt);

    void copy_to_particles(std::vector<Particle*> &particles);
};


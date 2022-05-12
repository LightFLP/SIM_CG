#pragma once

#include <Solver.h>
#include <Particle.h>
#include <Constraint.h>
#include <linearSolver.h>

struct GlobalVars{
    int n; // particles
    // state vectors of size 2*n
    double* x;
    double* v;
    double* Q;
    
    // state vector of size n (technically 2*n, but repetition)
    double* W;

    int m; // constraints
    // state vectors of size m
    double* C;
    double* Cdot;

    GlobalVars(int _n, int _m){
        n = _n;
        m = _m;
        
        x = (double*) malloc(sizeof(double) * 2 * n);
        v = (double*) malloc(sizeof(double) * 2 * n);
        Q = (double*) malloc(sizeof(double) * 2 * n);
        W = (double*) malloc(sizeof(double) * n);

        C = (double*) malloc(sizeof(double) * m);
        Cdot = (double*) malloc(sizeof(double) * m);
    }

    GlobalVars(const GlobalVars& other){
        n = other.n;
        m = other.m;
        
        x = (double*) malloc(sizeof(double) * 2 * n);
        v = (double*) malloc(sizeof(double) * 2 * n);
        Q = (double*) malloc(sizeof(double) * 2 * n);
        W = (double*) malloc(sizeof(double) * n);

        C = (double*) malloc(sizeof(double) * m);
        Cdot = (double*) malloc(sizeof(double) * m);

        std::memcpy(x, other.x, sizeof(double)*2*n);
        std::memcpy(v, other.v, sizeof(double)*2*n);
        std::memcpy(Q, other.Q, sizeof(double)*2*n);
        std::memcpy(W, other.W, sizeof(double)*n);

        std::memcpy(C, other.C, sizeof(double)*m);
        std::memcpy(C, other.Cdot, sizeof(double)*m);
    }

    ~GlobalVars(){
        free(x);
        free(v);
        free(Q);
        free(W);
        free(C);
        free(Cdot);
    }
};

class State{
    GlobalVars globals;
    int n, m;

    std::vector<Particles*>* particles;
    std::vector<Constraint*>* constraints;
    std::vector<Force*>* forces;

    implicitMatrixWithTrans* J;
    implicitMatrixWithTrans* Jdot;
    implicitJWJt* JWjt;

    double* lambda;
    
    // setup RHS of:
    // JWJt * lambda = - Jq - JWQ - alphaC - betaCdot
	double*  Jq;
	double* JWQ;
	double* ksC;
	double* kdC;
    double* RHS;
public:
    State(std::vector<Particle*>* _particles, 
          std::vector<Constraint*>* _constraints, 
          std::vector<Force*>* _forces){
        int n = particles.size();
        int m = constraints.size();
    	
        globals = GlobalVars();

        particles = _particles;
        constraints = _constraints;
        forces = _forces;

        // initialise matrices
        J = new implicitMatrixWithTrans(m, 2*n);
        Jdot = new implicitMatrixWithTrans(m, 2*n);
        JWJt = new implicitJWJt(J);
        JWJt->W = (double*) malloc(sizeof(double) * n * 2);
        int i = 0;
        for (Particle* p : particles){
            JWJt->W[i++] = 1/p->m_Mass;
            JWJt->W[i++] = 1/p->m_Mass;
        } 

        // we need to make the matrices and constraints share pointers to matrix blocks
        for (Constraint *c : constraints){
            for (MatrixBlock* mb : c->matrix_blocks_J){
                J->blocks.push_back(mb);
            }
            for (MatrixBlock* mb : c->matrix_blocks_Jdot){
                Jdot->blocks.push_back(mb);
            }
	    }

        lambda = (double*) malloc(sizeof(double) * m); // allocate space for lambda

        // setup global_RHS
        Jq  = (double*) malloc(sizeof(double) * m);
        JWQ = (double*) malloc(sizeof(double) * m);
        ksC = (double*) malloc(sizeof(double) * m);
        kdC = (double*) malloc(sizeof(double) * m);
        RHS = (double*) malloc(sizeof(double) * m);
    }

    State(State& other){
        
    }
    
    ~State(){
        free(globals);
    }

    void advance(double dt){
        // We follow the steps here from the slides

        // 1. Clear forces
        for (Particle* p : particles) p->m_ForceAccum = Vec2(0, 0);

        // 2a. Accumulate forces
        for (Force* f : forces) f->calculate_forces();
        // 2b. Put these in globals
        int i = 0;
        for (Particle* p : particles){
            globals.Q[i++] = p->m_ForceAccum[0];
            globals.Q[i++] = p->m_ForceAccum[1];
        }

        // 3. Solve constraint forces
        // 3a. first evaluate all constraint functions
        std::memset(globals.C, 0.0, m); // clear old values first
	    std::memset(globals.Cdot, 0.0, m);
        Constraint* c;
        for (i = 0; i < m; i++){
            c = constraints[i];
            globals.C[i] += c->eval_C();
            globals.Cdot[i] += c->eval_Cdot();
            c->eval_J();
            c->eval_Jdot();
        }

        // 3b. Calculate RHS
        Jdot->matVecMult(globals.v, Jq); // J. * q.

        vecMultComp(globals.Q, globals.W); // W*Q
        J->matVecMult(globals.Q, JWQ); // J*W*Q

        
    }

    State evaluate(double dt){
        
    }
};
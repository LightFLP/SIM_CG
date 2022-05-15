#include "Simulator.h"

void State::setup_globals(std::vector<Particle*> &particles){
    int i = 0;
    for (Particle* p : particles){
        globals->x[i] = p->m_Position[0];
        globals->v[i] = p->m_Velocity[0];
        globals->W[i] = 1/p->m_Mass;
        printf("i: %i x: %.2f v: %.2f W: %.2f\n", i, globals->x[i], globals->v[i], globals->W[i]);
        i++;
        globals->x[i] = p->m_Position[1];
        globals->v[i] = p->m_Velocity[1];
        globals->W[i] = 1/p->m_Mass;
        printf("i: %i x: %.2f v: %.2f W: %.2f\n", i, globals->x[i], globals->v[i], globals->W[i]);
        i++;
    }
}

void State::reset(std::vector<Particle*> &particles){
        memset(globals->data, 0.0, 8*n + 2*m);
        setup_globals(particles);
}

State::State(Solver* _solver, int _n, int _m, std::vector<Particle*> &particles) : n(_n), m(_m){
    solver = _solver;
    J =    new implicitMatrixWithTrans(m, 2*n);
    Jdot = new implicitMatrixWithTrans(m, 2*n);
    JWJt = new implicitJWJt(J);

    globals = new GlobalVars(n, m);

    setup_globals(particles);

    // initialise matrices
//     std::memcpy(JWJt->W, globals->W, sizeof(double)*2*n);
        JWJt->W = globals->W;

    lambda = (double*) malloc(sizeof(double) * m); // allocate space for lambda
    std::memset(lambda, 0.0, sizeof(double) * m);
    // setup global_RHS
    Jq  = (double*) malloc(sizeof(double) * m);
    JWQ = (double*) malloc(sizeof(double) * m);
    ksC = (double*) malloc(sizeof(double) * m);
    kdC = (double*) malloc(sizeof(double) * m);
    RHS = (double*) malloc(sizeof(double) * m);
}

GlobalVars* State::evaluate(double dt, Solver* eval_solver){
    GlobalVars* new_globals = new GlobalVars(globals);
    if(!eval_solver){
        SympleticEulerSolver().simulation_step(new_globals, dt);
    } else {
        eval_solver->simulation_step(new_globals, dt);
    }
    return new_globals;
}


void State::advance(double dt, std::vector<Particle*> &particles, 
                               std::vector<Constraint*> &constraints, 
                               std::vector<Force*> &forces, 
                               std::vector<Force*> &mouse_forces){
#ifdef DEBUG
        printf("\n---------------[STATE::ADVANCE]----------------\n");
#endif
        // We follow the steps here from the slides
        // 1. Clear forces
        int i;
        
        std::memset(globals->Q, 0.0, sizeof(double) * 2*n);
        // 2a. Accumulate forces
        for (Force* f : forces) f->calculate_forces(globals);
        for (Force* f : mouse_forces) f->calculate_forces(globals);
#ifdef DEBUG
        for (i= 0; i < 2*n; i++){
            printf("\tQ[%i] = %.2f\n", i, globals->Q[i]);
        }
        printf("2a\n");
#endif
        // 3. Solve constraint forces
        // 3a. first evaluate all constraint functions
        // clear old values first
        std::memset(globals->C, 0.0, sizeof(double) * m); 
	std::memset(globals->Cdot, 0.0, sizeof(double) * m);
        J->Clear();
        Jdot->Clear();
        Constraint* c;
        for (i = 0; i < m; i++){
            c = constraints[i];
            globals->C[i] += c->eval_C(globals);
            globals->Cdot[i] += c->eval_Cdot(globals);
            c->eval_J(globals, J->blocks);
            //printf("J->blocks last = %i\n", J->blocks.back().data);
            c->eval_Jdot(globals, Jdot->blocks);
        }
#ifdef DEBUG
        printf("\t MATRIX    J: \n");
        J->print_matrix();
        printf("\t MATRIX Jdot: \n");
        Jdot->print_matrix();

        for (i= 0; i < m; i++){
            printf("\tC[%i]    = %.2f\n", i, globals->C[i]);
            printf("\tCdot[%i] = %.2f\n", i, globals->Cdot[i]);
        }
        printf("3a\n");
#endif

        // 3b. Calculate RHS
        Jdot->matVecMult(globals->v, Jq); // J. * q.
#ifdef DEBUG
        printf("JDot\n");
        for (i= 0; i < 2*n; i++){
            printf("\tQ[%i] = %.2f\n", i, globals->Q[i]);
        }
        for (i= 0; i < 2*n; i++){
            printf("\tW[%i] = %.2f\n", i, globals->W[i]);
        }
#endif
        vecMultComp(2*n, globals->Q, globals->W); // W*Q
#ifdef DEBUG
        for (i= 0; i < 2*n; i++){
            printf("\tWQ[%i] = %.2f\n", i, globals->Q[i]);
        }
        //for (i = 0; i < 8*n + 2*m; i++) printf("\t\t data[%i] = %.2f\n", i, globals->data[i]);
        std::cout << J << " " << J->nrows << " " << J->ncols << std::endl;
#endif
        J->matVecMult(globals->Q, JWQ); // J*W*Q
#ifdef DEBUG
        for (i= 0; i < m; i++){
            printf("\tJWQ[%i] = %.2f\n", i, JWQ[i]);
        }
#endif
        vecTimesScalar(m, globals->C, alpha); // damping
        vecTimesScalar(m, globals->Cdot, beta); // damping
#ifdef DEBUG
        printf("alpha\n");
        printf("beta\n");
#endif

        // Assemble RHS
        vecAssign(m, RHS, Jq);
#ifdef DEBUG
        printf("RHS = Jq\n");
        for (i = 0; i < m; i++){
            printf("\tRHS[%i]=%.2f\n", i, RHS[i]);
        }
#endif
        vecAddEqual(m, RHS, JWQ);
#ifdef DEBUG
        printf("RHS += JWQ\n");
        for (i = 0; i < m; i++){
            printf("\tRHS[%i]=%.2f\n", i, RHS[i]);
        }
#endif
        vecAddEqual(m, RHS, globals->C);
#ifdef DEBUG
        printf("RHS += kdC\n");
        for (i = 0; i < m; i++){
            printf("\tRHS[%i]=%.2f\n", i, RHS[i]);
        }
#endif
        vecAddEqual(m, RHS, globals->Cdot);
#ifdef DEBUG
        printf("RHS += ksC\n");
        for (i = 0; i < m; i++){
            printf("\tRHS[%i]=%.2f\n", i, RHS[i]);
        }
#endif
        vecTimesScalar(m, RHS, -1); //needs to be negative
#ifdef DEBUG
        printf("RHS *= -1\n");
        for (i = 0; i < m; i++){
            printf("\tRHS[%i]=%.2f\n", i, RHS[i]);
        }
#endif
        // 3c. Solve system
        int steps = 0;
#ifdef DEBUG
        for (i = 0; i < m; i++){
            printf("\tLamb[%i]=%.2f\n", i, lambda[i]);
        }
        for (i = 0; i < 2*n; i++){
            printf("\t JWJt->W[%i] = %.5f, W[%i] = %.5f\n", i, JWJt->W[i], i, globals->W[i]);
        }
        printf("\t MATRIX JWJt\n");
        JWJt->print_matrix();
#endif
        ConjGrad(m, JWJt, lambda, RHS, 1e-25, &steps);
#ifdef DEBUG
        for (i = 0; i < m; i++){
            printf("\tLamb[%i]=%.2f\n", i, lambda[i]);
        }
#endif
        // Obtain Qhat through Jt * lambda = Qhat
        double* Qhat = (double*) malloc(sizeof(double) * 2 * n);
        J->matTransVecMult(lambda, Qhat);
#ifdef DEBUG
        for (i = 0; i < 2*n; i++){
            printf("\tQhat[%i]=%.2f\n", i, Qhat[i]);
        }
#endif
        // Since Q contains W*Q, and we want to add Qhat to make it (Q + Qhat)*W, we do Qhat*W first
        vecMultComp(2*n, Qhat, globals->W);
        vecAddEqual(2*n, globals->Q, Qhat);
        free(Qhat);

        // 4. Throw all this new found information to our solver
        solver->simulation_step(globals, dt);
    }

    void State::copy_to_particles(std::vector<Particle*> &particles){
        Particle* p;
        for (int i = 0; i < n; i++){
            p = particles[i];
            p->m_Position[0] = globals->x[2*i];
            p->m_Position[1] = globals->x[2*i+1];
            // Technically we only need position I think
            // (velocity/force is used for drawing)
            p->m_Velocity[0] = globals->v[2*i];
            p->m_Velocity[1] = globals->v[2*i+1];
            p->m_ForceAccum[0] = globals->Q[2*i]*p->m_Mass;
            p->m_ForceAccum[1] = globals->Q[2*i+1]*p->m_Mass;
        }
    }
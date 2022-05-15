#pragma once
#include "Solver.h"

#include <vector>


class EulerSolver : public Solver{
	public:
		virtual void simulation_step( GlobalVars* globals, float dt ){
			for(int i = 0; i < 2*globals->n; i++){
				globals->x[i] += dt*globals->v[i];
				globals->v[i] += dt*globals->Q[i];
			}
		}
};


class SympleticEulerSolver : public Solver{
	public:
		virtual void simulation_step( GlobalVars* globals, float dt ){
#ifdef DEBUG
			printf(" SymplEuler, before:\n");
			for (int i = 0; i < 8*globals->n + 2*globals->m; i++){
				printf("\tdata[%i]=%.3f\n", i, globals->data[i]);
			}
#endif
			for(int i = 0; i < 2*globals->n; i++){
				globals->v[i] += dt*globals->Q[i];
				globals->x[i] += dt*globals->v[i];
			}
#ifdef DEBUG
			printf(" SymplEuler, after:\n");
			for (int i = 0; i < 8*globals->n + 2*globals->m; i++){
				printf("\tdata[%i]=%.3f\n", i, globals->data[i]);
			}
#endif
		}
};


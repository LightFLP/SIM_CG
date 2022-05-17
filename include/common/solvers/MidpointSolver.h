#pragma once
#include "Solver.h"

class MidpointSolver : public Solver{
	public:
		virtual void simulation_step( GlobalVars* globals, float dt ){
			GlobalVars* midpoint_globals = 
            
            for(int i = 0; i < 2*globals->n; i++){
				globals->x[i] += dt*globals->v[i];
                
				globals->v[i] += dt*globals->Q[i];
			}
		}
};
#include "Solver.h"
#include "Simulator.h" //State import

class RK4Solver : public Solver{
	public:
		void simulation_step( State* state, double dt );
};


#include "Solver.h"
class State;
#include "EulerSolvers.h"

class RK4Solver : public Solver{
	State *state_k2 = nullptr;
	State *state_k3 = nullptr;
	State *state_k4 = nullptr;
	Solver* solver = new SympleticEulerSolver();
	public:
		void simulation_step( State* state, double dt );
};


#pragma once
#include "EulerSolvers.h"

class MidpointSolver : public Solver{
	public:
		Solver* solver = new EulerSolver();
		MidpointSolver(Solver* _solver = nullptr);
		State* midpoint_state = nullptr;
		virtual void simulation_step( State* state, double dt );
};

class SympleticMidpointSolver : public Solver{
	public:
		Solver* solver = new SympleticEulerSolver();
		SympleticMidpointSolver(Solver* _solver = nullptr);
		State* midpoint_state = nullptr;
		virtual void simulation_step( State* state, double dt );
};
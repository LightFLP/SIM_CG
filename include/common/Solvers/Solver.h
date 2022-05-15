#pragma once
#include "GlobalVars.h"

class Solver{
	public:
		virtual void simulation_step(GlobalVars* globals, float dt) = 0;
};


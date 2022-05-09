#pragma once
#include "Particle.h"

#include <vector>


class Solver{
	public:
		virtual void simulation_step(std::vector<Particle*> pVector, float dt);
};


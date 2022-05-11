#include "Solver.h"

#include <vector>

#include <valarray>

typedef std::vector<Vec2> MathVec;


class RK4Solver : public Solver{

	// Solving [dx/dt, dv/dt] = [v, a]
	// X = [x, v]
	MathVec RHS(Particle* p, MathVec X){
		MathVec output(2);
		output[0] = X[1];
		output[1] = p->m_ForceAccum/p->m_Mass;
		return output;
	}

	public:
		virtual void simulation_step( std::vector<Particle*> pVector, float dt )
		{
			for(Particle* p : pVector)
			{
				MathVec X = {p->m_Position, p->m_Velocity};
				MathVec k1 = RHS(p, X);
				MathVec k2 = RHS(p, {X[0] + dt/2.0 * k1[0], X[1] + dt/2.0 * k1[1]});
				MathVec k3 = RHS(p, {X[0] + dt/2.0 * k2[0], X[1] + dt/2.0 * k2[1]});
				MathVec k4 = RHS(p, {X[0] + dt * k3[0], X[1] + dt * k3[1]});
				X[0] = 1 / 6.0 * (k1[0] + 2*(k2[0]+k3[0]) + k4[0]);
				X[1] = 1 / 6.0 * (k1[1] + 2*(k2[1]+k3[1]) + k4[1]);
				p->m_Position += dt*X[0];
				p->m_Velocity += dt*X[1];
			}
		}
};


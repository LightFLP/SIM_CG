#include "Solver.h"

#include <vector>


class EulerSolver : public Solver{
	public:
		virtual void simulation_step( std::vector<Particle*> pVector, float dt )
		{
			int ii, size = pVector.size();
			
			for(ii=0; ii<size; ii++)
			{
				pVector[ii]->m_Position += dt*pVector[ii]->m_Velocity;
				pVector[ii]->m_Velocity += dt*pVector[ii]->m_ForceAccum/pVector[ii]->m_Mass;
			}

		}
};


class SympleticEulerSolver : public Solver{
	public:
		virtual void simulation_step( std::vector<Particle*> pVector, float dt )
		{
			int ii, size = pVector.size();
			
			for(ii=0; ii<size; ii++)
			{
				pVector[ii]->m_Velocity += dt*pVector[ii]->m_ForceAccum/pVector[ii]->m_Mass;
				pVector[ii]->m_Position += dt*pVector[ii]->m_Velocity;
			}

		}
};


#include "Solver.h"

#include <vector>

#include <valarray>

typedef std::vector<Vec2> MathVec;


class RK4Solver : public Solver{

	

	public:
		void simulation_step( GlobalVars* globals, float dt )
		{
			for (int i = 0; i < globals->n*2; i++)
			{
				
			}
		}
};


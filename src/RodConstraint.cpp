#include "RodConstraint.h"
#include <GL/glut.h>

RodConstraint::RodConstraint(Particle *p1, Particle *p2, int p_index1, int p_index2, int c_index, double dist)
 : m_dist(dist){
   pVector.push_back(p1);
   pVector.push_back(p2);
   matrix_blocks_J.push_back(new MatrixBlock(c_index, 2*p_index1, 1, 2));
   matrix_blocks_J.push_back(new MatrixBlock(c_index, 2*p_index2, 1, 2));
   matrix_blocks_Jdot.push_back(new MatrixBlock(c_index, 2*p_index1, 1, 2));
   matrix_blocks_Jdot.push_back(new MatrixBlock(c_index, 2*p_index2, 1, 2));
}

void RodConstraint::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( pVector[0]->m_Position[0], pVector[0]->m_Position[1] );
  glColor3f(0.8, 0.7, 0.6);
  glVertex2f( pVector[1]->m_Position[0], pVector[1]->m_Position[1] );
  glEnd();

}

double RodConstraint::eval_C(){
	return (pVector[0]->m_Position - pVector[1]->m_Position) * (pVector[0]->m_Position - pVector[1]->m_Position) - m_dist * m_dist;
}


double RodConstraint::eval_Cdot(){
	Particle* p1 = pVector[0];
  Particle* p2 = pVector[1];
  return 2*((p1->m_Velocity[0] - p2->m_Velocity[0])*(p1->m_Position[0] - p2->m_Position[0]) + 
            (p1->m_Velocity[1] - p2->m_Velocity[1])*(p1->m_Position[1] - p2->m_Position[1]) );
}

void RodConstraint::eval_J(){
	// dC/dp0
  matrix_blocks_J[0]->data[0] = 2*(pVector[0]->m_Position[0]-pVector[1]->m_Position[0]);
  matrix_blocks_J[0]->data[1] = 2*(pVector[0]->m_Position[1]-pVector[1]->m_Position[1]);
	// dC/dp1
  matrix_blocks_J[1]->data[0] = 2*(pVector[1]->m_Position[0]-pVector[0]->m_Position[0]);
	matrix_blocks_J[1]->data[1] = 2*(pVector[1]->m_Position[1]-pVector[0]->m_Position[1]);
}

// Jdot = Cdot/dparticles
void RodConstraint::eval_Jdot(){
	matrix_blocks_Jdot[0]->data[0] = 2*(pVector[0]->m_Velocity[0] - pVector[1]->m_Velocity[0]);
	matrix_blocks_Jdot[0]->data[1] = 2*(pVector[0]->m_Velocity[1] - pVector[1]->m_Velocity[1]);

  matrix_blocks_Jdot[1]->data[0] = 2*(pVector[1]->m_Velocity[0] - pVector[0]->m_Velocity[0]);
	matrix_blocks_Jdot[1]->data[1] = 2*(pVector[1]->m_Velocity[1] - pVector[0]->m_Velocity[1]);
}

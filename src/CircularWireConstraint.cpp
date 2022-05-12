#include "CircularWireConstraint.h"
#include <GL/glut.h>
#include <gfx/vec2.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2 & vect, float radius)
{
	glBegin(GL_LINE_LOOP);
	glColor3f(0.0,1.0,0.0); 
	for (int i=0; i<360; i=i+18)
	{
		float degInRad = i*PI/180;
		glVertex2f(vect[0]+cos(degInRad)*radius,vect[1]+sin(degInRad)*radius);
	}
	glEnd();
}

CircularWireConstraint::CircularWireConstraint(Particle* p, int p_index, int c_index, const Vec2 & center, const double radius) :
	m_center(center), m_radius(radius) {
		pVector.push_back(p);
		matrix_blocks_J.push_back(new MatrixBlock(c_index, 2*p_index, 1, 2));
		matrix_blocks_Jdot.push_back(new MatrixBlock(c_index, 2*p_index, 1, 2));
	}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

double CircularWireConstraint::eval_C(){
	return (pVector[0]->m_Position - m_center)*(pVector[0]->m_Position - m_center)-m_radius*m_radius;
}

// (x - c)^2 + (y - c)^2 -> 2(x-c)*x' + 2(y-c)*y'
double CircularWireConstraint::eval_Cdot(){
	return 2*(pVector[0]->m_Position - m_center)*pVector[0]->m_Velocity;
}


void CircularWireConstraint::eval_J(){
	matrix_blocks_J[0]->data[0] = 2*(pVector[0]->m_Position[0]-m_center[0]);
	matrix_blocks_J[0]->data[1] = 2*(pVector[0]->m_Position[1]-m_center[1]);
}

// Jdot = Cdot/dq
void CircularWireConstraint::eval_Jdot(){
	matrix_blocks_Jdot[0]->data[0] = 2*pVector[0]->m_Velocity[0];
	matrix_blocks_Jdot[0]->data[1] = 2*pVector[0]->m_Velocity[1];
}
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

CircularWireConstraint::CircularWireConstraint(int p_index, int c_index, const Vec2 center, const double radius) :
	m_center(center), m_radius(radius) {
		m_c_index = c_index;
		iVector.push_back(p_index);
	}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

double CircularWireConstraint::eval_C(GlobalVars* globals){
	Vec2 pos = globals->get_pos(iVector[0]);
#ifdef DEBUG
	printf("pos=(%.2f, %.2f) center=(%.2f, %.2f) r=%.2f => C=%.2f\n", pos[0], pos[1], m_center[0], m_center[1], m_radius, (pos - m_center)*(pos - m_center)-m_radius*m_radius);
#endif
	return (pos - m_center)*(pos - m_center)-m_radius*m_radius;
}

// (x - c)^2 + (y - c)^2 -> 2(x-c)*x' + 2(y-c)*y'
double CircularWireConstraint::eval_Cdot(GlobalVars* globals){
	Vec2 pos = globals->get_pos(iVector[0]);
	Vec2 vel = globals->get_vel(iVector[0]);
	return 2*((pos - m_center)*vel);
}

void CircularWireConstraint::eval_J(GlobalVars* globals, std::vector<MatrixBlock> & blocks){
	Vec2 pos = globals->get_pos(iVector[0]);
	MatrixBlock mb = MatrixBlock(m_c_index, iVector[0]*2, 1, 2);
	mb.data[0] = 2*(pos[0]-m_center[0]);
	mb.data[1] = 2*(pos[1]-m_center[1]);
	blocks.emplace_back(mb);
}

// Jdot = Cdot/dq
void CircularWireConstraint::eval_Jdot(GlobalVars* globals, std::vector<MatrixBlock> & blocks){
	Vec2 vel = globals->get_vel(iVector[0]);
	MatrixBlock mb = MatrixBlock(m_c_index, iVector[0]*2, 1, 2);
	mb.data[0] = 2*vel[0];
	mb.data[1] = 2*vel[1];
	blocks.emplace_back(mb);
}
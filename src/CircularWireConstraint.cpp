#include "CircularWireConstraint.h"
#include <GL/glut.h>
#include <gfx/vec2.h>

#define PI 3.1415926535897932384626433832795

static void draw_circle(const Vec2f & vect, float radius)
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

CircularWireConstraint::CircularWireConstraint(const Vec2f & center, const double radius) :
	m_center(center), m_radius(radius) {}

void CircularWireConstraint::draw()
{
	draw_circle(m_center, m_radius);
}

void CircularWireConstraint::calculate_forces(){
	for (Particle* p : pVector){
		// Lambda from ref [1], F3, 2.1
		float l = ( -p->m_ForceAccum * p->m_Position - p->m_Mass*p->m_Velocity*p->m_Velocity)/(p->m_Position*p->m_Position);
		Vec2f f_hat = l*p->m_Position;
		p->m_ForceAccum += f_hat;
	}
}
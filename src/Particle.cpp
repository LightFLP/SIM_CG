#include "Particle.h"
#include <GL/glut.h>

#define VIS_MULT 0.01

Particle::Particle(const Vec2f & ConstructPos) :
	m_ConstructPos(ConstructPos), m_Position(Vec2f(0.0, 0.0)), m_Velocity(Vec2f(0.0, 0.0))
{
}

Particle::~Particle(void)
{
}

void Particle::reset()
{
	m_Position = m_ConstructPos;
	m_Velocity = Vec2f(0.0, 0.0);
	m_ForceAccum = 0;
}

void Particle::draw(bool show_velocity, bool show_force)
{
    if (show_velocity) {
        glColor3f(1.f, 0.f, 0.f);
        glBegin(GL_LINES);
        glVertex2f(m_Position[0], m_Position[1]);
        glVertex2f(m_Position[0] + m_Velocity[0] * VIS_MULT, m_Position[1] + m_Velocity[1] * VIS_MULT);
        glEnd();
    }

    if (show_force) {
        glColor3f(0.f, 0.f, 1.f);
        glBegin(GL_LINES);
        glVertex2f(m_Position[0], m_Position[1]);
        glVertex2f(m_Position[0] + m_ForceAccum[0] * VIS_MULT, m_Position[1] + m_ForceAccum[1] * VIS_MULT);
        glEnd();
    }

	const double h = 0.03;
	glColor3f(1.f, 1.f, 1.f); 
	glBegin(GL_QUADS);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]-h/2.0);
	glVertex2f(m_Position[0]+h/2.0, m_Position[1]+h/2.0);
	glVertex2f(m_Position[0]-h/2.0, m_Position[1]+h/2.0);
	glEnd();
}

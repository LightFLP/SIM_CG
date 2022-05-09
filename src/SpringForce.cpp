#include "SpringForce.h"
#include <GL/glut.h>

SpringForce::SpringForce(Particle *p1, Particle * p2, double dist, double ks, double kd) :
  m_p1(p1), m_p2(p2), m_dist(dist), m_ks(ks), m_kd(kd) {}

void SpringForce::calculate_forces(){
  Vec2f l = m_p1->m_Position - m_p2->m_Position;
  Vec2f dldt = m_p1->m_Velocity - m_p2->m_Velocity;
  float l_mag = sqrt(l*l);
  Vec2f fa = -(m_ks*(l_mag - m_dist)+m_kd*(dldt*l)/l_mag)*l/l_mag;
  Vec2f fb = -fa;
  m_p1->m_ForceAccum += fa;
  m_p2->m_ForceAccum += fb;
}

void SpringForce::draw()
{
  glBegin( GL_LINES );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p1->m_Position[0], m_p1->m_Position[1] );
  glColor3f(0.6, 0.7, 0.8);
  glVertex2f( m_p2->m_Position[0], m_p2->m_Position[1] );
  glEnd();
}

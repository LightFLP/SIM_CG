#include "MouseSpringForce.h"

#include <GL/glut.h>
#include "GlobalVars.h"
#include "Particle.h"

MouseSpringForce::MouseSpringForce(int p_index1, Particle *_mouse_particle, double dist, double ks, double kd) :
        m_dist(dist), m_ks(ks), m_kd(kd) {
    register_particle(p_index1);
    mouse_p = _mouse_particle;
}

void MouseSpringForce::calculate_forces(GlobalVars *globals) {
    p0 = globals->get_pos(iVector[0]);
    p1 = mouse_p->m_Position;
    Vec2 v0 = globals->get_vel(iVector[0]);
    Vec2 v1 = mouse_p->m_Velocity;
    Vec2 l = p0 - p1;
    Vec2 dldt = v0 - v1;
    float l_mag = sqrt(l * l);
    Vec2 fa = -(m_ks * (l_mag - m_dist) + m_kd * (dldt * l) / l_mag) * l / l_mag;
    fa = fa * fa * fa * fa * fa + 0.5 * fa; // x^5 + 0.5x
    globals->Q[iVector[0] * 2] += fa[0];
    globals->Q[iVector[0] * 2 + 1] += fa[1];
}

void MouseSpringForce::draw() {
    glBegin(GL_LINES);
    glColor3f(0.0, 0.0, 0.0);
    glVertex2f(p0[0], p0[1]);
    glColor3f(1.0, 0.0, 0.0);
    glVertex2f(p1[0], p1[1]);
    glEnd();
}

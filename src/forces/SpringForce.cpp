#include "SpringForce.h"

#include <GL/glut.h>
#include "GlobalVars.h"

SpringForce::SpringForce(int p_index1, int p_index2, double dist, double ks, double kd) :
        m_dist(dist), m_ks(ks), m_kd(kd) {
    register_particle(p_index1);
    register_particle(p_index2);
}

void SpringForce::calculate_forces(GlobalVars *globals) {
    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    Vec2 v0 = globals->get_vel(iVector[0]);
    Vec2 v1 = globals->get_vel(iVector[1]);
#ifdef DEBUG
    printf("p0 = (%.2f, %.2f) p1 = (%.2f, %.2f) v0 = (%.2f, %.2f) v1 = (%.2f, %.2f)\n", p0[0], p0[1], p1[0], p1[1], v0[0], v0[1], v1[0], v1[1]);
#endif
    Vec2 l = p0 - p1;
    Vec2 dldt = v0 - v1;
    float l_mag = sqrt(l * l);
    Vec2 fa = -(m_ks * (l_mag - m_dist) + m_kd * (dldt * l) / l_mag) * (l / l_mag);
    Vec2 fb = -fa;
    globals->Q[iVector[0] * 2] += fa[0];
    globals->Q[iVector[0] * 2 + 1] += fa[1];
    globals->Q[iVector[1] * 2] += fb[0];
    globals->Q[iVector[1] * 2 + 1] += fb[1];
    fraction = m_dist / l_mag;
}

void SpringForce::draw() {
    fraction = fraction * fraction * 0.9 + 0.1;
    glBegin(GL_LINES);
    glColor3f(1.0 - fraction, fraction, 0.3);
    glVertex2f(p0[0], p0[1]);
    glColor3f(1.0 - fraction, fraction, 0.3);
    glVertex2f(p1[0], p1[1]);
    glEnd();
}

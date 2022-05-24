#include "RodConstraint.h"

#include <GL/glut.h>
#include "GlobalVars.h"

RodConstraint::RodConstraint(int p_index1, int p_index2, double dist) : m_dist(dist) {
    iVector.push_back(p_index1);
    iVector.push_back(p_index2);
}

void RodConstraint::draw() {
    glBegin(GL_LINES);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(p0[0], p0[1]);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(p1[0], p1[1]);
    glEnd();

}

double RodConstraint::eval_C(GlobalVars *globals) {
    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    return (p0 - p1) * (p0 - p1) - m_dist * m_dist;
}


double RodConstraint::eval_Cdot(GlobalVars *globals) {
    Vec2 v0 = globals->get_vel(iVector[0]);
    Vec2 v1 = globals->get_vel(iVector[1]);
    return (v0 - v1) * (v0 - v1);
}

void RodConstraint::eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    MatrixBlock mb0 = MatrixBlock(m_c_index, iVector[0] * 2, 1, 2);
    MatrixBlock mb1 = MatrixBlock(m_c_index, iVector[1] * 2, 1, 2);

    // dC/dp0
    mb0.data[0] = 2 * (p0[0] - p1[0]);
    mb0.data[1] = 2 * (p0[1] - p1[1]);
    // dC/dp1
    mb1.data[0] = 2 * (p1[0] - p0[0]);
    mb1.data[1] = 2 * (p1[1] - p0[1]);

    blocks.emplace_back(mb0);
    blocks.emplace_back(mb1);
}

// Jdot = Cdot/dparticles
void RodConstraint::eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
    Vec2 v0 = globals->get_vel(iVector[0]);
    Vec2 v1 = globals->get_vel(iVector[1]);
    MatrixBlock mb0 = MatrixBlock(m_c_index, iVector[0] * 2, 1, 2);
    MatrixBlock mb1 = MatrixBlock(m_c_index, iVector[1] * 2, 1, 2);

    mb0.data[0] = 2 * (v0[0] - v1[0]);
    mb0.data[1] = 2 * (v0[1] - v1[1]);

    mb1.data[0] = 2 * (v1[0] - v0[0]);
    mb1.data[1] = 2 * (v1[1] - v0[1]);

    blocks.emplace_back(mb0);
    blocks.emplace_back(mb1);
}

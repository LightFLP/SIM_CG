#include "WireConstraint.h"

#include <GL/glut.h>
#include <gfx/vec2.h>

#include "GlobalVars.h"

WireConstraint::WireConstraint(int p_index, const double height) : m_Height(height) {
    iVector.push_back(p_index);
}

void WireConstraint::draw() {
    glBegin(GL_LINES);
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f(-1.0, m_Height);
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f(1.0, m_Height);
    glEnd();
}

double WireConstraint::eval_C(GlobalVars *globals) {
    Vec2 pos = globals->get_pos(iVector[0]);
    return pos[1] - m_Height;
}

double WireConstraint::eval_Cdot(GlobalVars *globals) {
    Vec2 vel = globals->get_vel(iVector[0]);
    return vel[1];
}

void WireConstraint::eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
    MatrixBlock mb = MatrixBlock(m_c_index, iVector[0] * 2, 1, 2);
    mb.data[0] = 0;
    mb.data[1] = 1;
    blocks.emplace_back(mb);
}

void WireConstraint::eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
    MatrixBlock mb = MatrixBlock(m_c_index, iVector[0] * 2, 1, 2);
    mb.data[0] = 0;
    mb.data[1] = 0;
    blocks.emplace_back(mb);
}

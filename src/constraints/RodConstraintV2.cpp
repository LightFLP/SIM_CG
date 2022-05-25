#include "RodConstraintV2.h"

#include <GL/glut.h>
#include "GlobalVars.h"

RodConstraintV2::RodConstraintV2(int p_index1, int p_index2, double dist) : m_dist(dist) {
    iVector.push_back(p_index1);
    iVector.push_back(p_index2);
}

void RodConstraintV2::draw() {
    glBegin(GL_LINES);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(p0[0], p0[1]);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(p1[0], p1[1]);
    glEnd();

}

double RodConstraintV2::eval_C(GlobalVars *globals) {
    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    return sqrt((p0 - p1) * (p0 - p1)) - m_dist;
}


double RodConstraintV2::eval_Cdot(GlobalVars *globals) {
    Vec2 v0 = globals->get_vel(iVector[0]);
    Vec2 v1 = globals->get_vel(iVector[1]);
    return ( (p0 - p1) * (v0 - v1) ) / (sqrt((p0 - p1) * (p0 - p1)));
}

void RodConstraintV2::eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    MatrixBlock mb0 = MatrixBlock(m_c_index, iVector[0] * 2, 1, 2);
    MatrixBlock mb1 = MatrixBlock(m_c_index, iVector[1] * 2, 1, 2);

    // dC/dp0
    mb0.data[0] = (p0[0] - p1[0]) / (sqrt((p0[0] - p1[0]) * (p0[0] - p1[0])));
    mb0.data[1] = (p0[1] - p1[1]) / (sqrt((p0[1] - p1[1]) * (p0[1] - p1[1])));
    // dC/dp1
    mb1.data[0] = - (p0[0] - p1[0]) / (sqrt((p0[0] - p1[0]) * (p0[0] - p1[0])));
    mb1.data[1] = - (p0[1] - p1[1]) / (sqrt((p0[1] - p1[1]) * (p0[1] - p1[1])));

    blocks.emplace_back(mb0);
    blocks.emplace_back(mb1);
}

// Jdot = Cdot/dparticles
void RodConstraintV2::eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
    Vec2 v0 = globals->get_vel(iVector[0]);
    Vec2 v1 = globals->get_vel(iVector[1]);

    MatrixBlock mb0 = MatrixBlock(m_c_index, iVector[0] * 2, 1, 2);
    MatrixBlock mb1 = MatrixBlock(m_c_index, iVector[1] * 2, 1, 2);

    mb0.data[0] = ((v0[0] - v1[0]) * ((2 * p0[0] - p1[0] * p0[0] - p1[0]) / sqrt(p0[0] - p1[0] * p0[0] - p1[0]))) / ( p0[0] - p1[0] * p0[0] - p1[0]);
    mb0.data[1] = ((v0[1] - v1[1]) * ((2 * p0[1] - p1[1] * p0[1] - p1[1]) / sqrt(p0[1] - p1[1] * p0[1] - p1[1]))) / ( p0[1] - p1[1] * p0[1] - p1[1]);

    mb1.data[0] = - ((v0[0] - v1[0]) * ((2 * p0[0] - p1[0] * p0[0] - p1[0]) / sqrt(p0[0] - p1[0] * p0[0] - p1[0]))) / ( p0[0] - p1[0] * p0[0] - p1[0]);
    mb1.data[1] = - ((v0[1] - v1[1]) * ((2 * p0[1] - p1[1] * p0[1] - p1[1]) / sqrt(p0[1] - p1[1] * p0[1] - p1[1]))) / ( p0[1] - p1[1] * p0[1] - p1[1]);

    blocks.emplace_back(mb0);
    blocks.emplace_back(mb1);
}

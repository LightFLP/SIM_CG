#include "AngularConstraint.h"

#include <GL/glut.h>
#include "GlobalVars.h"

#define PI 3.1415926535897932384626433832795

AngularConstraint::AngularConstraint(int p_index1, int p_index2, int p_index3, double ra) : m_ra(ra) {
    iVector.push_back(p_index1);
    iVector.push_back(p_index2);
    iVector.push_back(p_index3);
}

void AngularConstraint::draw() {
    glBegin(GL_LINES);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(p0[0], p0[1]);
    glColor3f(0.8, 0.7, 0.6);
    glVertex2f(p1[0], p1[1]);
    glEnd();

}
// DONE
double AngularConstraint::eval_C(GlobalVars *globals) {
    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    p2 = globals->get_pos(iVector[2]);

    Vec2 l1 = p0 - p1;
    Vec2 l2 = p2 - p1;

    double l1_mag = sqrt(l1 * l1);
    double l2_mag = sqrt(l2 * l2);

    double curCos = (l1 * l2) / (l1_mag * l2_mag);
    //curCos = (p0 - p1) * (p2 - p1) / (sqrt(p0 - p1 * p0 - p1) * sqrt(p2 - p1 * p2 - p1));

    float degInRad = m_ra * PI / 180;
    double cosSubtAngle = cos(degInRad);

    return (curCos * curCos) - (cosSubtAngle * cosSubtAngle);
}


double AngularConstraint::eval_Cdot(GlobalVars *globals) {
//    p0 = globals->get_pos(iVector[0]);
//    p1 = globals->get_pos(iVector[1]);
//    p2 = globals->get_pos(iVector[2]);
//
//    Vec2 v0 = globals->get_vel(iVector[0]);
//    Vec2 v1 = globals->get_vel(iVector[1]);
//    Vec2 v2 = globals->get_vel(iVector[2]);

    //double curCosDot = (v0 - v1) * (v2 - v1) / ((v0 - v1) / 2 * sqrt(p0 - p1) * sqrt(p0 - p1) * sqrt(p0 - p1) ) * ((v2 - v1) / 2 * sqrt(p2 - p1) * sqrt(p2 - p1) * sqrt(p2 - p1) );

    return 0.0;
}
// DONE
void AngularConstraint::eval_J(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    p2 = globals->get_pos(iVector[2]);
    MatrixBlock mb0 = MatrixBlock(m_c_index, iVector[0] * 2, 1, 2);
    MatrixBlock mb1 = MatrixBlock(m_c_index, iVector[2] * 2, 1, 2);

    //curCos = (p0 - p1) * (p2 - p1) / (sqrt(p0 - p1 * p0 - p1) * sqrt(p2 - p1 * p2 - p1));

    // dC/dp0
    mb0.data[0] = 2 * ((p0[0] - p1[0] * p2[0] - p1[0]) / (sqrt(p0[0] - p1[0] * p0[0] - p1[0]) * sqrt(p2[0] - p1[0] * p2[0] - p1[0])));
    mb0.data[1] = 2 * ((p0[1] - p1[1] * p2[1] - p1[1]) / (sqrt(p0[1] - p1[1] * p0[1] - p1[1]) * sqrt(p2[1] - p1[1] * p2[1] - p1[1])));
    // dC/dp1
    mb1.data[0] = 2 * ((p1[0] - p0[0] * p1[0] - p2[0]) / (sqrt(p1[0] - p0[0] * p1[0] - p0[0]) * sqrt(p1[0] - p2[0] * p1[0] - p2[0])));
    mb1.data[1] = 2 * ((p1[1] - p0[1] * p1[1] - p2[1]) / (sqrt(p1[1] - p0[1] * p1[1] - p0[1]) * sqrt(p1[1] - p2[1] * p1[1] - p2[1])));

    blocks.emplace_back(mb0);
    blocks.emplace_back(mb1);
}

// Jdot = Cdot/dparticles
void AngularConstraint::eval_Jdot(GlobalVars *globals, std::vector <MatrixBlock> &blocks) {
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

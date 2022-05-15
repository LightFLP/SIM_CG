#include "WireConstraint.h"
#include <gfx/vec2.h>
#include <GL/glut.h>

#define PI 3.1415926535897932384626433832795

inline const double deg_to_rad(double deg) {
    return deg * PI / 180;
}

WireConstraint::WireConstraint(int p_index, int c_index, const double angle, Vec2 center)
    : m_Center(center), m_Angle(angle) {
    m_c_index = c_index;
    iVector.push_back(p_index);
}

void WireConstraint::draw()
{
    glBegin( GL_LINES );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( -1.0 - m_Center[0], (-1.0 - m_Center[0]) * tan(deg_to_rad(m_Angle)) );
    glColor3f(0.6, 0.7, 0.8);
    glVertex2f( 1.0 - m_Center[0], (1.0 - m_Center[0]) * tan(deg_to_rad(m_Angle)) );
    glEnd();
}

double WireConstraint::eval_C(GlobalVars* globals){
    Vec2 pos = globals->get_pos(iVector[0]);
    return ((pos[1] - m_Center[1]) / cos(deg_to_rad(m_Angle))) + ((pos[0] -
    m_Center[0]) / cos(deg_to_rad(m_Angle)));
}

double WireConstraint::eval_Cdot(GlobalVars* globals){
    Vec2 pos = globals->get_pos(iVector[0]);
    Vec2 vel = globals->get_vel(iVector[0]);
    return ((pos[1] - m_Center[1]) / cos(deg_to_rad(m_Angle))) * vel[1]
    + ((pos[0] - m_Center[0]) / cos(deg_to_rad(m_Angle))) * vel[0];
}

void WireConstraint::eval_J(GlobalVars* globals, std::vector<MatrixBlock> & blocks){
    MatrixBlock mb = MatrixBlock(m_c_index, iVector[0], 1, 2);
    mb.data[0] = 1;
    mb.data[1] = 1;
    blocks.emplace_back(mb);
}

void WireConstraint::eval_Jdot(GlobalVars* globals, std::vector<MatrixBlock> & blocks){
    MatrixBlock mb = MatrixBlock(m_c_index, iVector[0], 1, 2);
    mb.data[0] = 0;
    mb.data[1] = 0;
    blocks.emplace_back(mb);
}

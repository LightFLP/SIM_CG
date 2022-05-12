#include "WireConstraint.h"
#include <gfx/vec2.h>
#include <GL/glut.h>

#define PI 3.1415926535897932384626433832795

inline const double deg_to_rad(double deg) {
    return deg * PI / 180;
}

WireConstraint::WireConstraint(Particle* p, int p_index, int c_index, const double angle)
    : m_Center(p->m_Position), m_Angle(angle) {
    pVector.push_back(p);
    matrix_blocks_J.push_back(new MatrixBlock(c_index, 2 * p_index, 1, 2));
    matrix_blocks_Jdot.push_back(new MatrixBlock(c_index, 2 * p_index, 1, 2));
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

double WireConstraint::eval_C(){
    return ((pVector[0]->m_Position[1] - m_Center[1]) / cos(deg_to_rad(m_Angle))) + ((pVector[0]->m_Position[0] -
    m_Center[0]) / cos(deg_to_rad(m_Angle)));
}

double WireConstraint::eval_Cdot(){
    return ((pVector[0]->m_Position[1] - m_Center[1]) / cos(deg_to_rad(m_Angle))) * pVector[0]->m_Velocity[1]
    + ((pVector[0]->m_Position[0] - m_Center[0]) / cos(deg_to_rad(m_Angle))) * pVector[0]->m_Velocity[0];
}

void WireConstraint::eval_J(){
    matrix_blocks_J[0]->data[0] = 1;
    matrix_blocks_J[0]->data[1] = 1;
}

void WireConstraint::eval_Jdot(){
    matrix_blocks_Jdot[0]->data[0] = 0;
    matrix_blocks_Jdot[0]->data[1] = 0;
}


//double WireConstraint::eval_C(){
//    return pVector[0]->m_Position[1] - m_Center[1];
//}
//
//double WireConstraint::eval_Cdot(){
//    return pVector[0]->m_Velocity[1];
//}
//
//void WireConstraint::eval_J(){
//    matrix_blocks_J[0]->data[0] = 0;
//    matrix_blocks_J[0]->data[1] = 1;
//}
//
//void WireConstraint::eval_Jdot(){
//    matrix_blocks_Jdot[0]->data[0] = 0;
//    matrix_blocks_Jdot[0]->data[1] = 0;
//}

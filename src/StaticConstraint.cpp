#include "StaticConstraint.h"
#include <gfx/vec2.h>

StaticConstraint::StaticConstraint(Particle* p, int p_index, int c_index) : m_StaticPos(p->m_Position) {
    pVector.push_back(p);
    matrix_blocks_J.push_back(new MatrixBlock(c_index, 2 * p_index, 1, 2));
    matrix_blocks_Jdot.push_back(new MatrixBlock(c_index, 2 * p_index, 1, 2));
}

void StaticConstraint::draw()
{
}

double StaticConstraint::eval_C(){
    return (pVector[0]->m_Position - m_StaticPos) * (pVector[0]->m_Position - m_StaticPos);
}

double StaticConstraint::eval_Cdot(){
    return 2 * (pVector[0]->m_Position - m_StaticPos) * pVector[0]->m_Velocity;
}

void StaticConstraint::eval_J(){
    matrix_blocks_J[0]->data[0] = 2 * (pVector[0]->m_Position[0] - m_StaticPos[0]);
    matrix_blocks_J[0]->data[1] = 2 * (pVector[0]->m_Position[1] - m_StaticPos[1]);
}

void StaticConstraint::eval_Jdot(){
    matrix_blocks_Jdot[0]->data[0] = 0;
    matrix_blocks_Jdot[0]->data[1] = 0;
//    matrix_blocks_Jdot[0]->data[0] = 2*pVector[0]->m_Velocity[0];
//    matrix_blocks_Jdot[0]->data[1] = 2*pVector[0]->m_Velocity[1];
}

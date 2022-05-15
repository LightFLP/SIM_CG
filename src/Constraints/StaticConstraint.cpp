#include "StaticConstraint.h"
#include <gfx/vec2.h>

StaticConstraint::StaticConstraint(int p_index, int c_index, Vec2 static_pos) : m_StaticPos(static_pos) {
    m_c_index = c_index;
    iVector.push_back(p_index);
}

void StaticConstraint::draw()
{
}

double StaticConstraint::eval_C(GlobalVars* globals){
    Vec2 pos = globals->get_pos(iVector[0]);
    return (pos - m_StaticPos) * (pos - m_StaticPos);
}

double StaticConstraint::eval_Cdot(GlobalVars* globals){
    Vec2 pos = globals->get_pos(iVector[0]);
    Vec2 vel = globals->get_vel(iVector[0]);
    return 2 * (pos - m_StaticPos) * vel;
}

void StaticConstraint::eval_J(GlobalVars* globals, std::vector<MatrixBlock> & blocks){
    Vec2 pos = globals->get_pos(iVector[0]);
    MatrixBlock mb = MatrixBlock(m_c_index, iVector[0]*2, 1, 2);
    
    mb.data[0] = 2 * (pos[0] - m_StaticPos[0]);
    mb.data[1] = 2 * (pos[1] - m_StaticPos[1]);

    blocks.emplace_back(mb);
}

void StaticConstraint::eval_Jdot(GlobalVars* globals, std::vector<MatrixBlock> & blocks){
    MatrixBlock mb = MatrixBlock(m_c_index, iVector[0]*2, 1, 2);
    mb.data[0] = 0;
    mb.data[1] = 0;
//    matrix_blocks_Jdot[0]->data[0] = 2*pVector[0]->m_Velocity[0];
//    matrix_blocks_Jdot[0]->data[1] = 2*pVector[0]->m_Velocity[1];
    blocks.emplace_back(mb);
}

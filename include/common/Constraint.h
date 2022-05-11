#pragma once

#include <Particle.h>
#include <vector>
#include <linearSolver.h>

class Constraint{
    protected:
        std::vector<Particle*> pVector = std::vector<Particle*>();
    public:
        std::vector<MatrixBlock*> matrix_blocks_J = std::vector<MatrixBlock*>();
        std::vector<MatrixBlock*> matrix_blocks_Jdot = std::vector<MatrixBlock*>();
        virtual void draw(){};
        virtual double eval_C()=0;
        virtual double eval_Cdot()=0;
        virtual void eval_J()=0;
        virtual void eval_Jdot()=0;
};
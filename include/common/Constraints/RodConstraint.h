#pragma once

#include <Constraint.h>

class RodConstraint : public Constraint {
 public:
  RodConstraint(int p_index1, int p_index2, int c_index, double dist);
  virtual void draw();
  virtual double eval_C(GlobalVars* globals);
  virtual double eval_Cdot(GlobalVars* globals);
    virtual void eval_J(GlobalVars* globals, std::vector<MatrixBlock> & blocks);
    virtual void eval_Jdot(GlobalVars* globals, std::vector<MatrixBlock> & blocks);

 private:
  double const m_dist;
  Vec2 p0, p1;
};

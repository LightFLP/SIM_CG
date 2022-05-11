#pragma once

#include "Particle.h"
#include "Constraint.h"

class RodConstraint : public Constraint {
 public:
  RodConstraint(Particle *p1, Particle * p2, int p_index1, int p_index2, int c_index, double dist);
  virtual void draw();
  virtual double eval_C();
  virtual double eval_Cdot();
  virtual void eval_J();
  virtual void eval_Jdot();

 private:
  double const m_dist;
};

#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(Particle* p, int p_index, int c_index, const Vec2 & center, const double radius);

  virtual void draw();

  virtual double eval_C();
  virtual double eval_Cdot();
  virtual void eval_J();
  virtual void eval_Jdot();
 private:
  Vec2 const m_center;
  double const m_radius;
};

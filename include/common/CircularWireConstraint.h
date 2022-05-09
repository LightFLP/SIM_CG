#pragma once

#include "Particle.h"
#include "Constraint.h"

class CircularWireConstraint : public Constraint {
 public:
  CircularWireConstraint(const Vec2f & center, const double radius);

  virtual void draw();
  virtual void calculate_forces();
 private:
  Vec2f const m_center;
  double const m_radius;
};

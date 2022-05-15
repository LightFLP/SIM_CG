#pragma once

#include "Particle.h"
#include "Force.h"

class SpringForce : public Force {
 public:
  SpringForce(int p_index1, int p_index2, double dist, double ks, double kd);

  virtual void draw();
  virtual void calculate_forces(GlobalVars* globals);
 private:
  Vec2 p0, p1;
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};

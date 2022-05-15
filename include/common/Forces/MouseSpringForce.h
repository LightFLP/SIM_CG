#pragma once

#include "Particle.h"
#include "Force.h"

class MouseSpringForce : public Force {
 public:
  MouseSpringForce(int p_index1, Particle* mouse_particle, double dist, double ks, double kd);

  virtual void draw();
  virtual void calculate_forces(GlobalVars* globals);
 private:
  Vec2 p0, p1;
  Particle* mouse_p;
  double const m_dist;     // rest length
  double const m_ks, m_kd; // spring strength constants
};
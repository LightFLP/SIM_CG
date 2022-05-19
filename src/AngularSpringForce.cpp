#include "AngularSpringForce.h"
#include <GL/glut.h>

AngularSpringForce::AngularSpringForce(int p_index1, int p_index2, int p_index3, double dist, double ks, double kd) :
m_dist(dist), m_ks(ks), m_kd(kd) {
  register_particle(p_index1);
  register_particle(p_index2);
}
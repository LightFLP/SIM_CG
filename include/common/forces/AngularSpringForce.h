#pragma once

#include <gfx/vec2.h>

#include "Force.h"

class AngularSpringForce : public Force {
    Vec2 p0, p1, p2;
    double const m_ra;       // rest angle
    double const m_dist;     // rest distance
    double const m_ks, m_kd; // spring strength constants
    double fraction;         // fraction of the rest length over the distance between particles

public:
    AngularSpringForce(int p_index1, int p_index2, int p_index3, double ra, double ks, double kd);
    AngularSpringForce(int p_index1, int p_index2, int p_index3, double ra, double dist, double ks, double kd);

    virtual void draw();

    virtual void calculate_forces(GlobalVars *globals);
};

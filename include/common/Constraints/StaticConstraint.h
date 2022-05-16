#pragma once

#include "CircularWireConstraint.h"

class StaticConstraint : public CircularWireConstraint {
public:
    StaticConstraint(int p_index, int c_index, const Vec2 pos)
            : CircularWireConstraint(p_index, c_index, pos, 0) {};

    virtual void draw() {};
};
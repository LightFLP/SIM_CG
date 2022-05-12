#pragma once

#include <vector>

#include "Particle.h"

#include "Force.h"
#include "ConstantForce.h"
#include "DragForce.h"
#include "SpringForce.h"

#include "Constraint.h"
#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "StaticConstraint.h"
#include "WireConstraint.h"

class Scene {
public:
    static void loadDefault(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
    static void loadDoubleCircle(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
    static void loadCloth(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
};

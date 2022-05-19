#pragma once

#include <vector>

#include "Particle.h"

#include "forces/Force.h"
#include "forces/ConstantForce.h"
#include "forces/DragForce.h"
#include "forces/SpringForce.h"
#include "forces/MouseSpringForce.h"
#include "forces/WindForce.h"

#include "constraints/Constraint.h"
#include "constraints/CircularWireConstraint.h"
#include "constraints/RodConstraint.h"
#include "constraints/StaticConstraint.h"
#include "constraints/WireConstraint.h"

class Scene {
public:
    static void loadDefault(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind);
    static void loadDoubleCircle(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind);
    static void loadClothStatic(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind);
    static void loadClothWire(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind);
    static void loadHairStatic(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
};

#pragma once

#include <vector>

#include "Particle.h"

#include "Forces/Force.h"
#include "Forces/ConstantForce.h"
#include "Forces/DragForce.h"
#include "Forces/SpringForce.h"
#include "Forces/MouseSpringForce.h"

#include "Constraints/Constraint.h"
#include "Constraints/CircularWireConstraint.h"
#include "Constraints/RodConstraint.h"
#include "Constraints/StaticConstraint.h"
#include "Constraints/WireConstraint.h"

class Scene {
public:
    static void loadDefault(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
    static void loadDoubleCircle(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
    static void loadClothStatic(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
    static void loadClothWire(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
    static void loadHairStatic(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints);
};

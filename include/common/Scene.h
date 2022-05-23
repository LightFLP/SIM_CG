#pragma once

#include <vector>

class Particle;

class Force;
class ConstantForce;
class DragForce;
class SpringForce;
class AngularSpringForce;
class MouseSpringForce;
class WindForce;

class Constraint;
class CircularWireConstraint;
class RodConstraint;
class StaticConstraint;
class WireConstraint;

class Scene {
public:
    static void loadDefault(std::vector<Particle *> &pVector, bool *wind);

    static void loadDoubleCircle(std::vector<Particle *> &pVector, bool *wind);

    static void loadClothStatic(std::vector<Particle *> &pVector, bool *wind);

    static void loadClothWire(std::vector<Particle *> &pVector, bool *wind);

    static void loadHairStatic(std::vector<Particle *> &pVector, bool *wind);

    static void loadAngularSpring(std::vector<Particle *> &pVector, bool *wind);
};

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
    static void loadDefault(std::vector<Particle *> &pVector, bool *wind, double* dt);

    static void loadDoubleCircle(std::vector<Particle *> &pVector, bool *wind, double* dt);

    static void loadClothStatic(std::vector<Particle *> &pVector, bool *wind, double* dt);

    static void loadClothWire(std::vector<Particle *> &pVector, bool *wind, double* dt);

    static void loadHairStatic(std::vector<Particle *> &pVector, bool *wind, double* dt);

    static void loadAngularSpring(std::vector<Particle *> &pVector, bool *wind, double* dt);
};
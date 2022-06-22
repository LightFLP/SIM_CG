#pragma once

#include <vector>

class Particle;

class Force;
class ConstantForce;
class DragForce;
class SpringForce;
class AngularSpringForce;
class WallRepulsionForce;
class MouseSpringForce;
class WindForce;
class FluidForce;

class Constraint;
class CircularWireConstraint;
class RodConstraint;
class StaticConstraint;
class WireConstraint;

class Scene {
public:
    static void loadFluid(std::vector<Particle *> &pVector, bool *wind, bool *collision, double* dt, int N_f, float* u, float* v);

    static void loadDefault(std::vector<Particle *> &pVector, bool *wind, bool *collision, double* dt, int N_f, float* u, float* v);

    static void loadDoubleCircle(std::vector<Particle *> &pVector, bool *wind, bool *collision, double* dt, int N_f, float* u, float* v);

    static void loadClothStatic(std::vector<Particle *> &pVector, bool *wind, bool *collision, double* dt, int N_f, float* u, float* v);

    static void loadClothWire(std::vector<Particle *> &pVector, bool *wind, bool *collision, double* dt, int N_f, float* u, float* v);

    static void loadHairStatic(std::vector<Particle *> &pVector, bool *wind, bool *collision, double* dt, int N_f, float* u, float* v);

    static void loadAngularSpring(std::vector<Particle *> &pVector, bool *wind, bool *collision, double* dt, int N_f, float* u, float* v);
};
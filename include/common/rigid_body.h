#pragma once

#include <gfx/vec2.h>
#include <gfx/vec3.h>
#include <gfx/mat2.h>
#include <gfx/quat.h>

#include <vector>

class Particle;

class RigidBody {
public:

#define STATE_SIZE 9 // only 2d

    RigidBody(Vec2 _position, Vec2 _particles, float _mass);

//    std::vector<Particle *> particles;
    std::vector<int> indices;

    // constant
    const float mass;
    const Vec2 x_0; // start position
    float Ibody; // body space inertia
    float Ibody_inv; // inv body space inertia

    // state
    Vec2 x; // position
    Vec3 q; // rotation (q)
    Vec2 P; // linear momentum
    Vec2 L; // angular momentum

    // derived
    Mat2 I_inv; // inv inertia
    Mat2 R; // rotation matrix
    Vec2 v; // velocity
    Vec2 omega; // angular velocity

    // computed
    Vec2 force;
    Vec2 torque;

    void reset();

    void setState(double *state);
    double *getState();
    double *getDerivState();

    void compute_force_and_torque();

    static std::vector<RigidBody *> _bodies;

    static void AddRigidBody(RigidBody *body);
};

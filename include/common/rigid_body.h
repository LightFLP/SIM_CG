#pragma once

#include <gfx/vec2.h>

#include <vector>

class Particle;

class RigidBody {
public:
    RigidBody(Vec2 _position, Vec2 _particles, float _mass);

    std::vector<int> indices;

    Vec2 position;
    Vec2 velocity;

    Vec2 force;
    const float mass;

    void update(double dt);

    static std::vector<bool> _rigid_indices;
    static std::vector<RigidBody *> _bodies;

    static void AddRigidBody(RigidBody *body);
};

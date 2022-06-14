#include "Particle.h"

#include <GL/glut.h>

#include "rigid_body.h"

std::vector<RigidBody *> RigidBody::_bodies;
std::vector<bool> RigidBody::_rigid_indices;

void RigidBody::AddRigidBody(RigidBody *body) {
    RigidBody::_bodies.emplace_back(body);
}

RigidBody::RigidBody(Vec2 _position, Vec2 _particles, float _mass) : position(_position), mass(_mass) {
    float particleMass = mass / (_particles[0] * _particles[1]);

    const double h = 0.015;
    for (int i = 0; i < _particles[0]; i++) {
        for (int j = 0; j < _particles[1]; j++) {
            indices.push_back(Particle::_particles.size());
            RigidBody::_rigid_indices.push_back(true);
            Particle::_particles.push_back(new Particle(Vec2(position[0] + i * h, position[1] + j * h), particleMass));
        }
    }
}

void update(double dt) {

}
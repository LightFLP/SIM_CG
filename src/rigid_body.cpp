#include <GL/glut.h>

#include <math.h>

#include "Particle.h"

#include "rigid_body.h"

#define ZERO Vec2(0,0)
#define IDENTITY Mat2(0, 1, 0, 1);
#define e 2.71828182845904523536028747135266
#define PI 3.1415926535897932384626433832795

Vec2 cross(const Vec2& u, const Vec2& v) {
    return u[0] * v[1] - u[1] * v[0];
}

double dot(const Vec2& u, const Vec2& v) {
    return u[0] * v[0] + u[1] * v[1];
}

//https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
Vec2 rotate_vector_by_quaternion(const Vec2& v, const Vec3& q) {
    // Extract the vector part of the quaternion
    Vec2 u(q[0], q[1]);

    // Extract the scalar part of the quaternion
    float s = q[2];

    // partial Euler–Rodrigues formula
    return  2.0f * dot(u, v) * u
            + (s*s - dot(u, u)) * v
            + 2.0f * s * cross(u, v);
}

std::vector<RigidBody *> RigidBody::_bodies;

void RigidBody::AddRigidBody(RigidBody *body) {
    RigidBody::_bodies.emplace_back(body);
}

RigidBody::RigidBody(Vec2 _position, Vec2 _particles, float _mass) : x_0(_position), mass(_mass) {
    float particleMass = mass / _particles[0] * _particles[1];

    const double h = 0.015;
    for (int i = 0; i < _particles[0]; i++) {
        for (int j = 0; j < _particles[1]; j++) {
            indices.push_back(Particle::_particles.size());
            Particle::_particles.push_back(new Particle(Vec2(x_0[0] + i * h, x_0[1] + j * h), particleMass));
        }
    }

    // precompute Ibody_0 and Ibody_inv_0
//    Mat2 tmp = ident();
//    tmp(0,0) = (_particles[0] * h) * (_particles[0] * h);
//    tmp(1,1) = (_particles[1] * h) * (_particles[1] * h);
//    Ibody = mass * tmp;
//    invert(Ibody_inv, Ibody);

    // TODO this should prob be a matrix again, but try to figure out if we can just use a singular value instead

    //https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    Ibody = mass / 12.0 * (_particles[0] * _particles[0] + _particles[1] * _particles[1]);
    Ibody_inv = 1.0 / Ibody;

    // initial state
    reset();
}

void RigidBody::reset() {
    x = x_0;
    q = Vec3(0, 0, 0);
    P = ZERO;
    L = ZERO;

    I_inv = Ibody_inv;
    R = IDENTITY;
    std::cout << R << std::endl;
    v = ZERO;
    omega = ZERO;

    force = ZERO;
    torque = ZERO;

//    for (Particle *p : particles) {
//        p->reset();
//    }
}

void RigidBody::setState(double *state) {
    // position
    x[0] = state[0];
    x[1] = state[1];

    // rotation
    q[2] = state[2];
    q[0] = state[3];
    q[1] = state[4];

    // P
    P[0] = state[5];
    P[1] = state[6];

    // L
    L[0] = state[7];
    L[1] = state[8];

    // compute aux vars
    R = unit_quat_to_matrix(qdot); //TODO

    // https://www.essentialmath.com/GDC2013/GDC13_quaternions_final.pdf
    R = Mat2(cos(), -sin(), sin(), cos());
    v = P / mass;
    I_inv = R * Ibody_inv * transpose(R);
    omega = I_inv * L;

    // update particles
    for (int i = 0; i < indices.size(); i++) {
        Particle* p = Particle::_particles[indices[i]];

        p->m_Position = R * p->m_ConstructPos + x;
    }
}

double *RigidBody::getState() {
    double *state = (double *) malloc(sizeof(double) * STATE_SIZE);

    // position
    state[0] = x[0];
    state[1] = x[1];

    // rotation
    state[2] = q[2];
    state[3] = q[0];
    state[4] = q[1];

    // P
    state[5] = P[0];
    state[6] = P[1];

    // L
    state[7] = L[0];
    state[8] = L[1];

    return state;
}

double *RigidBody::getDerivState() {
    double *state = (double *) malloc(sizeof(double) * STATE_SIZE);
    compute_force_and_torque();

    // velocity
    state[0] = v[0];
    state[1] = v[1];

    // rotation
    Vec3 qdot = 0.5 * rotate_vector_by_quaternion(omega, q); // quaternion disguised as vector
    state[2] = qdot[2];
    state[3] = qdot[0];
    state[4] = qdot[1];

    // Pdot
    state[5] = force[0];
    state[6] = force[1];

    // Ldot
    state[7] = torque[0];
    state[8] = torque[1];

    return state;
}

void RigidBody::compute_force_and_torque() {
    torque = ZERO;
    force = ZERO;
    for (int i = 0; i < indices.size(); i++) {
        Particle* p = Particle::_particles[indices[i]];

        // t = r_x * f_y - r_y * f_x
        torque += (p->m_Position[0] - x[0]) * p->m_ForceAccum[1] - (p->m_Position[1] - x[1]) * p->m_ForceAccum[0];
        force = p->m_ForceAccum;
    }
}
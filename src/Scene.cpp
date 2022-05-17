#include "Scene.h"

#define sqrt2 1.41421356237

void Scene::loadDefault(std::vector<Particle*>& pVector, std::vector<Force*>& forces) {
    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offset(dist, 0.0);

    // Create three particles, attach them to each other, then add a
    // circular wire constraint to the first.

    pVector.push_back(new Particle(center + offset));
    pVector.push_back(new Particle(center + offset + offset ));
    pVector.push_back(new Particle(center + offset + offset + offset ));

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.0005)); // drag

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
    }

    forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));

    Constraint::addConstraint(new CircularWireConstraint(0, center, dist));
    Constraint::addConstraint(new RodConstraint(1, 2, dist));
}

void Scene::loadDoubleCircle(std::vector<Particle*>& pVector, std::vector<Force*>& forces) {
    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offset(dist, 0.0);

    pVector.push_back(new Particle(center + offset));
    pVector.push_back(new Particle(center + offset + offset ));
    pVector.push_back(new Particle(center + offset + offset + offset ));

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.0005)); // drag

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
    }

    forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));

    Constraint::addConstraint(new CircularWireConstraint(0, center, dist));
    Constraint::addConstraint(new CircularWireConstraint(1, 3 * offset, dist));
    Constraint::addConstraint(new RodConstraint(1, 2, dist));
}

void Scene::loadClothStatic(std::vector<Particle*>& pVector, std::vector<Force*>& forces) {

    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);

    const int N = 5;
    const double offset = N * dist / 2.0;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            Vec2 pos = Vec2(center[0] + j * dist, center[1] - i * dist);
            pVector.push_back(new Particle(Vec2(pos[0] - offset, pos[1] + offset)));
        }
    }

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.0005)); // drag

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
    }

    const double kd_structural = 50.0;
    const double ks_structural = 0.4;
    const double kd_shear = 1.0;
    const double ks_shear = 1.0;
    const double kd_bending = 20.0;
    const double ks_bending = 0.2;

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {

            // Structural springs (neighbours)
            if (i > 0) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - N, dist, kd_structural, ks_structural));
            }
            if (j > 0) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - 1, dist, kd_structural, ks_structural));
            }

            // Shear springs (diagonal)
            if (i > 0 && j > 0) {
                forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j - 1, dist * sqrt2, kd_shear, ks_shear));
            }
            if (i > 0 && j < N - 1) {
                forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j + 1, dist * sqrt2, kd_shear, ks_shear));
            }

            // Bending springs (second neighbours)
            if (i > 1) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - 2 * N, dist * 2, kd_bending, ks_bending));
            }
            if (j > 1) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - 2, dist * 2, kd_bending, ks_bending));
            }
        }
    }

    // Hanging points
    Constraint::addConstraint(new StaticConstraint(0, pVector[0]->m_ConstructPos));
    Constraint::addConstraint(new StaticConstraint(N - 1, pVector[N - 1]->m_ConstructPos));
}


void Scene::loadClothWire(std::vector<Particle*>& pVector, std::vector<Force*>& forces) {

    const double dist = 0.05;
    const Vec2 center(0.0, 0.0);

    const int N = 32;
    const double offset = N * dist / 2.0;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            Vec2 pos = Vec2(center[0] + j * dist, center[1] - i * dist);
            pVector.push_back(new Particle(Vec2(pos[0] - offset, pos[1] + offset)));
        }
    }

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.0005)); // drag

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
    }

    const double kd_structural = 50.0;
    const double ks_structural = 0.4;
    const double kd_shear = 1.0;
    const double ks_shear = 1.0;
    const double kd_bending = 20.0;
    const double ks_bending = 0.2;

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {

            // Structural springs (neighbours)
            if (i > 0) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - N, dist, kd_structural, ks_structural));
            }
            if (j > 0) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - 1, dist, kd_structural, ks_structural));
            }

            // Shear springs (diagonal)
            if (i > 0 && j > 0) {
                forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j - 1, dist * sqrt2, kd_shear, ks_shear));
            }
            if (i > 0 && j < N - 1) {
                forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j + 1, dist * sqrt2, kd_shear, ks_shear));
            }

            // Bending springs (second neighbours)
            if (i > 1) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - 2 * N, dist * 2, kd_bending, ks_bending));
            }
            if (j > 1) {
                forces.push_back(new SpringForce(N * i + j, N * i + j - 2, dist * 2, kd_bending, ks_bending));
            }
        }
    }

    // Wire
    for (int i = 0; i < N; i++) {
        Constraint::addConstraint(new WireConstraint(i, pVector[i]->m_ConstructPos[1]));
    }
}
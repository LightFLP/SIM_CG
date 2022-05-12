#include "../include/common/Scene.h"

void Scene::loadDefault(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints) {
    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offset(dist, 0.0);

    // Create three particles, attach them to each other, then add a
    // circular wire constraint to the first.

    pVector.push_back(new Particle(center + offset));
    pVector.push_back(new Particle(center + offset + offset ));
    pVector.push_back(new Particle(center + offset + offset + offset ));

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.05)); // drag

    for (Particle* p : pVector){
        forces[0]->register_particle(p); // gravity
        forces[1]->register_particle(p); // drag
    }

    forces.push_back(new SpringForce(pVector[0], pVector[1], dist, 500.0, 0.2));

    constraints.push_back(new CircularWireConstraint(pVector[0], 0, 0, center, dist));
    constraints.push_back(new RodConstraint(pVector[1], pVector[2], 1, 2, 1, dist));
}

void Scene::loadDoubleCircle(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints) {
    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offset(dist, 0.0);

    pVector.push_back(new Particle(center + offset));
    pVector.push_back(new Particle(center + offset + offset ));
    pVector.push_back(new Particle(center + offset + offset + offset ));

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.05)); // drag

    for (Particle* p : pVector){
        forces[0]->register_particle(p); // gravity
        forces[1]->register_particle(p); // drag
    }

    forces.push_back(new SpringForce(pVector[0], pVector[1], dist, 500.0, 0.2));

    constraints.push_back(new CircularWireConstraint(pVector[0], 0, 0, Vec2(0, 0), dist));
    constraints.push_back(new CircularWireConstraint(pVector[1], 1, 1, 3*offset, dist));
    constraints.push_back(new RodConstraint(pVector[1], pVector[2], 1, 2, 2, dist));
}

void Scene::loadCloth(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints) {

    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offsetX(dist, 0.0);
    const Vec2 offsetY(0.0, dist);

//    const int size_i = 3, size_j = 3;
//    for (int i = 0; i < size_i; i++) {
//        for (int j = 0; j < size_j; j++) {
//            pVector.push_back(new Particle(center + offsetX * i + offsetY * j));
//        }
//    }

    pVector.push_back(new Particle(center));
    pVector.push_back(new Particle(center - offsetY));

//    constraints.push_back(new StaticConstraint(pVector[0], 0, 0));
    constraints.push_back(new WireConstraint(pVector[0], 0, 0, 90));

    forces.push_back(new SpringForce(pVector[0], pVector[1], dist, 500.0, 0.2));

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.05)); // drag

    for (Particle* p : pVector) {
        forces[0]->register_particle(p); // gravity
        forces[1]->register_particle(p); // drag
    }

}
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
    forces.push_back(new DragForce(0.0005)); // drag

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
    }

    forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));

    constraints.push_back(new CircularWireConstraint(0, 0, center, dist));
    constraints.push_back(new RodConstraint(1, 2, 1, dist));
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

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
    }

    forces.push_back(new SpringForce(0, 1, dist, 500.0, 0.2));

    constraints.push_back(new CircularWireConstraint(0, 0, Vec2(0, 0), dist));
    constraints.push_back(new CircularWireConstraint(1, 1, 3*offset, dist));
    constraints.push_back(new RodConstraint(1, 2, 2, dist));
}

void Scene::loadCloth(std::vector<Particle*>& pVector, std::vector<Force*>& forces, std::vector<Constraint*>& constraints) {

    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offsetX(dist, 0.0);
    const Vec2 offsetY(0.0, dist);

    pVector.push_back(new Particle(center));
    pVector.push_back(new Particle(center - offsetY));

//    constraints.push_back(new StaticConstraint(pVector[0], 0, 0));
    constraints.push_back(new WireConstraint(0, 0, 0, 90));

    forces.push_back(new SpringForce(0, 1, dist, 500.0, 0.2));

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.05)); // drag

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
    }

}
#include "Scene.h"

#define sqrt2 1.41421356237

void Scene::loadDefault(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind) {
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
    forces.push_back(new WindForce(wind)); // wind

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
        forces[2]->register_particle(i); // wind
    }

    forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));

    Constraint::addConstraint(new CircularWireConstraint(0, center, dist));
    Constraint::addConstraint(new RodConstraint(1, 2, dist));
}

void Scene::loadDoubleCircle(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind) {
    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offset(dist, 0.0);

    pVector.push_back(new Particle(center + offset));
    pVector.push_back(new Particle(center + offset + offset ));
    pVector.push_back(new Particle(center + offset + offset + offset ));

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.0005)); // drag
    forces.push_back(new WindForce(wind)); // wind

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
        forces[2]->register_particle(i); // wind
    }

    forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));

    Constraint::addConstraint(new CircularWireConstraint(0, center, dist));
    Constraint::addConstraint(new CircularWireConstraint(1, 3 * offset, dist));
    Constraint::addConstraint(new RodConstraint(1, 2, dist));
}

void Scene::loadClothStatic(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind) {

    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);

    const int N = 9;
    const double offset = N * dist / 2.0;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            Vec2 pos = Vec2(center[0] + j * dist, center[1] - i * dist);
            pVector.push_back(new Particle(Vec2(pos[0] - offset, pos[1] + offset)));
        }
    }

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.0005)); // drag
    forces.push_back(new WindForce(wind)); // wind

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
        forces[2]->register_particle(i); // wind
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


void Scene::loadClothWire(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind) {

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
    forces.push_back(new WindForce(wind)); // wind

    for (int i = 0; i < pVector.size(); i++){
        forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
        forces[2]->register_particle(i); // wind
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

void Scene::loadHairStatic(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind) {

    const double dist = 0.1;
    const Vec2 center(0.0, +0.5f);
    const double angle_rad = 1.745;
    const double ks_angular = 0.5;
    const double kd_angular = 0.1;
    const double kd_spring = 50.0;
    const double ks_spring = 0.4;
    const int N = 9;
    int spring_forces_count = 0;

    for (int i = 0; i < N; i++) {

        if(i%2)
        {
            pVector.push_back(new Particle(Vec2(center[0] + 0.5, center[1] - i * dist)));
        }else{
            pVector.push_back(new Particle(Vec2(center[0], center[1] - i * dist)));
        }
    }

    forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    forces.push_back(new DragForce(0.0005)); // drag
    forces.push_back(new WindForce(wind)); // wind

    for (int i = 0; i < pVector.size(); i++){
        //forces[0]->register_particle(i); // gravity
        forces[1]->register_particle(i); // drag
        forces[2]->register_particle(i); // wind
    }

    for (int i = 0; i < N-2; i++) {
        forces.push_back(new AngularSpringForce(i, i+1, i+2, angle_rad, ks_angular, kd_angular)); // angular spring
        spring_forces_count += 1;
    }

    std::cout << "Nr of springs:" << spring_forces_count << std::endl;


    for (int i = 0; i < spring_forces_count; i++) {
        forces[3+i]->register_particle(i);
        forces[3+i]->register_particle(i+1);
        forces[3+i]->register_particle(i+2);
        std::cout << "Registering particles:" << i << "," << i+1 << "," << i+2 << ", to force:" << 3+i << std::endl;
    }

    for (int i = 0; i < pVector.size()-1; i++) {
        forces.push_back(new SpringForce(i, i+1, dist, kd_spring, ks_spring));
        std::cout << "Registering particles:" << i << "," << i+1 << " to spring force" << std::endl;
    }

    // Hanging point
    Constraint::addConstraint(new StaticConstraint(0, pVector[0]->m_ConstructPos));
}

void Scene::loadAngularSpring(std::vector<Particle*>& pVector, std::vector<Force*>& forces, bool *wind) {
    const double dist = 0.2;
    const double kd_structural = 50.0;
    const double ks_structural = 0.4;

    const Vec2 center(0.0, +0.5f);
    const Vec2 offset_left(-dist, 0.0);
    const Vec2 offset_right(dist, 0.0);

    std::cout << "Nr of particles:" << pVector.size() << std::endl;

    pVector.push_back(new Particle(offset_left));
    pVector.push_back(new Particle(center));
    pVector.push_back(new Particle(offset_right));
    std::cout << "Nr of particles:" << pVector.size() << std::endl;

    //forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    //forces.push_back(new DragForce(0.0005)); // drag
    forces.push_back(new WindForce(wind)); // wind
    forces.push_back(new AngularSpringForce(0, 1, 2, 0.5 , 0.5, 0.1)); // angular spring

    for (int i = 0; i < pVector.size(); i++){
//        forces[0]->register_particle(i); // gravity
//        forces[1]->register_particle(i); // drag
        forces[2]->register_particle(i); // wind
        forces[3]->register_particle(i); // angular spring force
    }

    forces.push_back(new SpringForce(0, 1, dist, kd_structural, ks_structural));
    forces.push_back(new SpringForce(1, 2, dist, kd_structural, ks_structural));

    // Hanging point
    Constraint::addConstraint(new StaticConstraint(0, pVector[0]->m_ConstructPos));
}



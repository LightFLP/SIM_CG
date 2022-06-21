#include "Scene.h"

#include "Particle.h"

#include "Force.h"
#include "ConstantForce.h"
#include "DragForce.h"
#include "SpringForce.h"
#include "AngularSpringForce.h"
#include "WallRepulsionForce.h"
#include "MouseSpringForce.h"
#include "WindForce.h"

#include "Constraint.h"
#include "CircularWireConstraint.h"
#include "RodConstraint.h"
#include "RodConstraintV2.h"
#include "StaticConstraint.h"
#include "WireConstraint.h"

#define sqrt2 1.41421356237

void Scene::loadDefault(std::vector<Particle*>& pVector, bool *wind, bool *collision, double* dt) {
    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offset(dist, 0.0);

    // Create three particles, attach them to each other, then add a
    // circular wire constraint to the first.

    pVector.push_back(new Particle(center + offset));
    pVector.push_back(new Particle(center + offset + offset ));
    pVector.push_back(new Particle(center + offset + offset + offset ));

    Force::_forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    Force::_forces.push_back(new DragForce(0.0005)); // drag
    Force::_forces.push_back(new WindForce(wind, dt)); // wind
    Force::_forces.push_back(new WallRepulsionForce(collision, -1, 1)); // wall

    for (int i = 0; i < pVector.size(); i++){
        Force::_forces[0]->register_particle(i); // gravity
        Force::_forces[1]->register_particle(i); // drag
        Force::_forces[2]->register_particle(i); // wind
        Force::_forces[3]->register_particle(i); // wind
    }

    Force::_forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));

    Constraint::addConstraint(new CircularWireConstraint(0, center, dist));
    Constraint::addConstraint(new RodConstraint(1, 2, dist));
}

void Scene::loadDoubleCircle(std::vector<Particle*>& pVector, bool *wind, bool *collision, double* dt) {
    const double dist = 0.2;
    const Vec2 center(0.0, 0.0);
    const Vec2 offset(dist, 0.0);

    pVector.push_back(new Particle(center + offset));
    pVector.push_back(new Particle(center + offset + offset ));
    pVector.push_back(new Particle(center + offset + offset + offset ));

    Force::_forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    Force::_forces.push_back(new DragForce(0.0005)); // drag
    Force::_forces.push_back(new WindForce(wind, dt)); // wind

    for (int i = 0; i < pVector.size(); i++){
        Force::_forces[0]->register_particle(i); // gravity
        Force::_forces[1]->register_particle(i); // drag
        Force::_forces[2]->register_particle(i); // wind
    }

    Force::_forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));

    Constraint::addConstraint(new CircularWireConstraint(0, center, dist));
    Constraint::addConstraint(new CircularWireConstraint(1, 3 * offset, dist));
    Constraint::addConstraint(new RodConstraint(1, 2, dist));
}

void Scene::loadClothStatic(std::vector<Particle*>& pVector, bool *wind, bool *collision, double* dt) {

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

    Force::_forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    Force::_forces.push_back(new DragForce(0.0005)); // drag
    Force::_forces.push_back(new WindForce(wind, dt)); // wind
    WallRepulsionForce* wall_collision = new WallRepulsionForce(collision, -1, 1);


    for (int i = 0; i < pVector.size(); i++){
        Force::_forces[0]->register_particle(i); // gravity
        Force::_forces[1]->register_particle(i); // drag
        Force::_forces[2]->register_particle(i); // wind
        wall_collision->register_particle(i); // wall collision
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
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - N, dist, kd_structural, ks_structural));
            }
            if (j > 0) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - 1, dist, kd_structural, ks_structural));
            }

            // Shear springs (diagonal)
            if (i > 0 && j > 0) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j - 1, dist * sqrt2, kd_shear, ks_shear));
            }
            if (i > 0 && j < N - 1) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j + 1, dist * sqrt2, kd_shear, ks_shear));
            }

            // Bending springs (second neighbours)
            if (i > 1) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - 2 * N, dist * 2, kd_bending, ks_bending));
            }
            if (j > 1) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - 2, dist * 2, kd_bending, ks_bending));
            }
        }
    }

    Force::_forces.push_back(wall_collision);

    // Hanging points
    Constraint::addConstraint(new StaticConstraint(0, pVector[0]->m_ConstructPos));
    Constraint::addConstraint(new StaticConstraint(N - 1, pVector[N - 1]->m_ConstructPos));
}


void Scene::loadClothWire(std::vector<Particle*>& pVector, bool *wind, bool *collision, double* dt) {

    const double dist = 0.05;
    const Vec2 center(0.0, 0.0);

    const int N = 8;
    const double offset = N * dist / 2.0;
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            Vec2 pos = Vec2(center[0] + j * dist, center[1] - i * dist);
            pVector.push_back(new Particle(Vec2(pos[0] - offset, pos[1] + offset)));
        }
    }

    Force::_forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    Force::_forces.push_back(new DragForce(0.0005)); // drag
    Force::_forces.push_back(new WindForce(wind, dt)); // wind
    WallRepulsionForce* wall_collision = new WallRepulsionForce(collision, -1, 1);

    for (int i = 0; i < pVector.size(); i++){
        Force::_forces[0]->register_particle(i); // gravity
        Force::_forces[1]->register_particle(i); // drag
        Force::_forces[2]->register_particle(i); // wind
        wall_collision->register_particle(i); // wall collision
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
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - N, dist, kd_structural, ks_structural));
            }
            if (j > 0) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - 1, dist, kd_structural, ks_structural));
            }

            // Shear springs (diagonal)
            if (i > 0 && j > 0) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j - 1, dist * sqrt2, kd_shear, ks_shear));
            }
            if (i > 0 && j < N - 1) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * (i - 1) + j + 1, dist * sqrt2, kd_shear, ks_shear));
            }

            // Bending springs (second neighbours)
            if (i > 1) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - 2 * N, dist * 2, kd_bending, ks_bending));
            }
            if (j > 1) {
                Force::_forces.push_back(new SpringForce(N * i + j, N * i + j - 2, dist * 2, kd_bending, ks_bending));
            }
        }
    }
    Force::_forces.push_back(wall_collision); // wall

    // Wire
    for (int i = 0; i < N; i++) {
        Constraint::addConstraint(new WireConstraint(i, pVector[i]->m_ConstructPos[1]));
    }
}

void Scene::loadHairStatic(std::vector<Particle*>& pVector, bool *wind, bool *collision, double* dt) {

    const Vec2 center(0.0, 0.0);

    const double dist = 0.05;
    const double angle = 45;
    const double ks_angular = 20;
    const double kd_angular = 0.2;
    const double kd_spring = 50.0;
    const double ks_spring = 0.4;
    const int N = 18;


    //Define forces
    Force::_forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
    Force::_forces.push_back(new DragForce(0.0005)); // drag
    Force::_forces.push_back(new WindForce(wind, dt)); // wind
    Force::_forces.push_back(new WallRepulsionForce(collision, -1, 1)); // wall

    //Face particles
    pVector.push_back(new Particle(Vec2(center[0] + 0.1 , center[1] + 0.05)));
    pVector.push_back(new Particle(Vec2(center[0] - 0.1 , center[1] + 0.05)));
    pVector.push_back(new Particle(Vec2(center[0] , center[1])));
    pVector.push_back(new Particle(Vec2(center[0] + 0.05 , center[1] - 0.09)));
    pVector.push_back(new Particle(Vec2(center[0] + 0.1 , center[1] - 0.04)));
    pVector.push_back(new Particle(Vec2(center[0] - 0.05 , center[1] - 0.09)));
    pVector.push_back(new Particle(Vec2(center[0] - 0.1 , center[1] - 0.04)));

    //Face constraints
    Constraint::addConstraint(new CircularWireConstraint(0, center, 0.2));

    for (int i = 0; i < 7; i++) {
        Constraint::addConstraint(new StaticConstraint(i, pVector[i]->m_ConstructPos));
    }
    Constraint::addConstraint(new RodConstraint(3, 4, dist));
    Constraint::addConstraint(new RodConstraint(3, 5, dist));
    Constraint::addConstraint(new RodConstraint(5, 6, dist));

    //Hair
    int particle_index = 7;
    for (int ep = 1; ep < 4; ep++) {

        Vec2 hair_center(0.2*ep - 0.4f, 0.17f);

        if(ep%2){
            hair_center = Vec2(0.2*ep - 0.4f, 0.0f);
        }


        int spring_forces_count = 0;

        int offset = 0;
        for (int i = particle_index; i < N+particle_index; i++) {

            if(i%2)
            {
                Vec2 pos = Vec2(hair_center[0], hair_center[1] - offset * dist);
                pVector.push_back(new Particle(Vec2(pos[0], pos[1] +0.04)));
            }else{
                Vec2 pos = Vec2(hair_center[0] + dist, hair_center[1] -offset * dist);
                pVector.push_back(new Particle(Vec2(pos[0] , pos[1] +0.04)));
            }
            offset+=1;
        }

        for (int i = particle_index; i < (N-2)+particle_index; i++) {
            Force::_forces.push_back(new AngularSpringForce(i, i+1, i+2, angle, ks_angular, kd_angular)); // angular spring
            spring_forces_count += 1;
        }

        //std::cout << "Nr of springs:" << spring_forces_count << std::endl;


        for (int i = particle_index; i < spring_forces_count; i++) {
            Force::_forces[3+i]->register_particle(i);
            Force::_forces[3+i]->register_particle(i+1);
            Force::_forces[3+i]->register_particle(i+2);
            //std::cout << "Registering particles:" << i << "," << i+1 << "," << i+2 << ", to force:" << 3+i << std::endl;
        }

        for (int i = particle_index; i < (N-1)+particle_index; i++) {
            Force::_forces.push_back(new SpringForce(i, i+1, dist, kd_spring, ks_spring));
            //std::cout << "Registering particles:" << i << "," << i+1 << " to spring force" << std::endl;
        }

        // Hanging point
        Constraint::addConstraint(new StaticConstraint(particle_index, pVector[particle_index]->m_ConstructPos));

        particle_index += N;
    }

    for (int i = 0; i < pVector.size(); i++){
        Force::_forces[0]->register_particle(i); // gravity
        Force::_forces[1]->register_particle(i); // drag
        Force::_forces[2]->register_particle(i); // wind
        Force::_forces[3]->register_particle(i); // wall collision
    }
}

void Scene::loadAngularSpring(std::vector<Particle*>& pVector, bool *wind, bool *collision, double* dt) {
    const double dist = 0.2;

    const double angle = 45;
    const double ks_angular = 20;
    const double kd_angular = 0.2;

    const double kd_structural = 0.4;
    const double ks_structural = 50;

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
    Force::_forces.push_back(new WindForce(wind, dt)); // wind
    Force::_forces.push_back(new WallRepulsionForce(collision, -1, 1)); // wall
    Force::_forces.push_back(new AngularSpringForce(0, 1, 2, angle , ks_angular, kd_angular)); // angular spring

    for (int i = 0; i < pVector.size(); i++){
//        forces[0]->register_particle(i); // gravity
//        forces[1]->register_particle(i); // drag
        Force::_forces[2]->register_particle(i); // wind
        Force::_forces[3]->register_particle(i); // wall collision
        Force::_forces[4]->register_particle(i); // angular spring force
    }

    Force::_forces.push_back(new SpringForce(0, 1, dist, ks_structural, kd_structural));
    Force::_forces.push_back(new SpringForce(1, 2, dist, ks_structural, kd_structural));

    // Hanging point
    Constraint::addConstraint(new StaticConstraint(0, pVector[0]->m_ConstructPos));
}

//void Scene::FluidCloth(std::vector<Particle*>& pVector, bool *wind, bool *collision, double* dt) {
//    const double dist = 0.2;
//    const Vec2 center(0.0, 0.0);
//    const Vec2 offset(dist, 0.0);
//
//    // Create three particles, attach them to each other, then add a
//    // circular wire constraint to the first.
//
//    pVector.push_back(new Particle(center + offset));
//    pVector.push_back(new Particle(center + offset + offset ));
//    pVector.push_back(new Particle(center + offset + offset + offset ));
//
//    Force::_forces.push_back(new ConstantForce(Vec2(0, -9.81))); // gravity
//    Force::_forces.push_back(new DragForce(0.0005)); // drag
//    Force::_forces.push_back(new WindForce(wind, dt)); // wind
//    Force::_forces.push_back(new WallRepulsionForce(collision, -1, 1)); // wall
//
//    for (int i = 0; i < pVector.size(); i++){
//        Force::_forces[0]->register_particle(i); // gravity
//        Force::_forces[1]->register_particle(i); // drag
//        Force::_forces[2]->register_particle(i); // wind
//        Force::_forces[3]->register_particle(i); // wind
//    }
//
//    Force::_forces.push_back(new SpringForce(0, 1, dist, 50.0, 0.2));
//
//    Constraint::addConstraint(new CircularWireConstraint(0, center, dist));
//    Constraint::addConstraint(new RodConstraint(1, 2, dist));
//
// /*
//    -------------------------------------------------------------------
//    Fluid simulation
//    -------------------------------------------------------------------
//*/
//
//}


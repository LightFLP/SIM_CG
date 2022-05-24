#include "AngularSpringForce.h"

#include <GL/glut.h>
#include "GlobalVars.h"

#define PI 3.1415926535897932384626433832795


AngularSpringForce::AngularSpringForce(int p_index1, int p_index2, int p_index3, double ra, double ks, double kd) :
        m_ra(ra), m_dist(0), m_ks(ks), m_kd(kd) {
    register_particle(p_index1);
    register_particle(p_index2);
    register_particle(p_index3);
}

void AngularSpringForce::calculate_forces(GlobalVars *globals) {

    p0 = globals->get_pos(iVector[0]);
    p1 = globals->get_pos(iVector[1]);
    p2 = globals->get_pos(iVector[2]);

    Vec2 v0 = globals->get_vel(iVector[0]);
    Vec2 v2 = globals->get_vel(iVector[2]);

#ifdef DEBUG
    Vec2 v1 = globals->get_vel(iVector[1]);
    printf("p0 = (%.2f, %.2f) p1 = (%.2f, %.2f) p2 = (%.2f, %.2f) v0 = (%.2f, %.2f) v1 = (%.2f, %.2f) v2 = (%.2f, %.2f)\n", p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], v0[0], v0[1], v1[0], v1[1],v2[0], v2[1]);
#endif

    Vec2 l1 = p0 - p1;
    Vec2 l2 = p2 - p1;



    // Apply cosine rule
    float degInRad = m_ra*PI/180;
    double cosSubtAngle = cos(degInRad);

    double b = sqrt(l1*l1);
    double c = sqrt(l2*l2);
    double r = sqrt(b * b + c * c - 2 * b * c * cosSubtAngle);

    // Compute spring force
    Vec2 l = p0 - p2;
    double l_mag = sqrt(l * l);
    Vec2 dldt = v0 - v2;

    Vec2 fa = -(m_ks * (l_mag - r) + m_kd * ((l * dldt) / l_mag)) * (l / l_mag);
    Vec2 fb = -fa;

//    std::cout << r << std::endl;

    globals->Q[iVector[0] * 2] += fa[0];
    globals->Q[iVector[0] * 2 + 1] += fa[1];
    globals->Q[iVector[2] * 2] += fb[0];
    globals->Q[iVector[2] * 2 + 1] += fb[1];
    fraction = r / l_mag;
}

// Controlling anisotropy way
//void AngularSpringForce::calculate_forces(GlobalVars* globals){
//
//    p0 = globals->get_pos(iVector[0]);
//    p1 = globals->get_pos(iVector[1]);
//    p2 = globals->get_pos(iVector[2]);
//
//    Vec2 v0 = globals->get_vel(iVector[0]);
//    Vec2 v1 = globals->get_vel(iVector[1]);
//    Vec2 v2 = globals->get_vel(iVector[2]);
//
//#ifdef DEBUG
//    printf("p0 = (%.2f, %.2f) p1 = (%.2f, %.2f) p2 = (%.2f, %.2f) v0 = (%.2f, %.2f) v1 = (%.2f, %.2f) v2 = (%.2f, %.2f)\n", p0[0], p0[1], p1[0], p1[1], p2[0], p2[1], v0[0], v0[1], v1[0], v1[1],v2[0], v2[1]);
//#endif
//
//    // Compute differences in positions
//    Vec2f l1 = p0 - p1;
//    Vec2f l2 = p2 - p1;
//    // Compute differences in velocities
//    Vec2f dl1dt = v0 - v1;
//    Vec2f dl2dt = v1 - v2;
//    // Calculate length of vectors
//    float l1_mag = sqrt(l1*l1);
//    float l2_mag = sqrt(l2*l2);
//
//    float c = cosf(m_ra);
//    float curC = (l1 * l2) / (l1_mag * l2_mag);
//
////    // Put spring force on first arm
////    Vec2 fa1 = -(m_ks*(l1_mag - m_dist)+m_kd*(dl1dt*l1)/l1_mag)*(l1/l1_mag);
////    Vec2 fb1 = -fa1;
////
////    globals->Q[iVector[0]*2] += fa1[0];
////    globals->Q[iVector[0]*2+1] += fa1[1];
////    globals->Q[iVector[1]*2] += fb1[0];
////    globals->Q[iVector[1]*2+1] += fb1[1];
////
////    // Put spring force on second arm
////    Vec2 fa2 = -(m_ks*(l2_mag - m_dist)+m_kd*(dl2dt*l2)/l2_mag)*(l2/l2_mag);
////    Vec2 fb2 = -fa2;
////
////    globals->Q[iVector[1]*2] += fa2[0];
////    globals->Q[iVector[1]*2+1] += fa2[1];
////    globals->Q[iVector[2]*2] += fb2[0];
////    globals->Q[iVector[2]*2+1] += fb2[1];
//
//    // Angular springs
//    Vec2f f1 = -m_ks * (curC - c) * l2 / l2_mag;
//    Vec2f f2 = -f1;
//    Vec2f f3 = -m_ks * (curC - c) * l1 / l1_mag;
//    Vec2f f4 = -f3;
//
//
//    globals->Q[iVector[0]*2] += f1[0];
//    globals->Q[iVector[0]*2+1] += f1[1];
//    globals->Q[iVector[1]*2] += f2[0];
//    globals->Q[iVector[1]*2+1] += f2[1];
//
//    globals->Q[iVector[1]*2] += f3[0];
//    globals->Q[iVector[1]*2+1] += f3[1];
//    globals->Q[iVector[2]*2] += f4[0];
//    globals->Q[iVector[2]*2+1] += f4[1];
//}

void AngularSpringForce::draw() {
    fraction = fraction * fraction * 0.9 + 0.1;
    glBegin(GL_LINES);
    glColor3f(1.0 - fraction, fraction, 0.3);
    // First line
    glVertex2f(p0[0], p0[1]);
    glVertex2f(p1[0], p1[1]);
    // Second line
    glVertex2f(p1[0], p1[1]);
    glVertex2f(p2[0], p2[1]);
    glEnd();
}

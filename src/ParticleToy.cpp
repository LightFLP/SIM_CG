// ParticleToy.cpp : Defines the entry point for the console application.
//
// #define DEBUG
// #define STEP
#include <gfx/geom3d.h>
#include <GL/glut.h>

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "Particle.h"
#include "imageio.h"
#include "EulerSolvers.h"
#include "RK4Solver.h"
#include "MidpointSolver.h"
#include "Scene.h"
#include "Force.h"
#include "Constraint.h"
#include "Simulator.h"
#include "MouseSpringForce.h"

/* macros */

Solver *solver = new MidpointSolver();
/* global variables */

static int N;
static double dt, d;
static int dsim;
static int dump_frames;
static int frame_number;
static int n_frames;
static int last_time;
static int dt_since_start = 0;
static float fps = 0.0f;
static double dts;

static int filesystem_opened;
static std::ofstream frametime_file;

static State *state;

static std::vector<Particle *> pVector;
static Particle *mouse_particle;
static int particle_selected = false;
static int scene_int = 1;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;

// booleans
static bool show_velocity = false;
static bool show_force = false;
static bool blow_wind = false;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data(void) {
    pVector.clear();
}

// number of constraints
static int m;
// number of particles
static int n;


static void init_system(void) {
    // Make sure the simulation is paused
    dsim = true;

    // Clear all lists
    pVector.clear();
    Force::_forces.clear();
    Force::_mouse_forces.clear();
    Constraint::_constraints.clear();

    // Load new scene
    switch (scene_int) {
        case 1:
            Scene::loadDefault(pVector, &blow_wind, &dts); break;
        case 2:
            Scene::loadDoubleCircle(pVector, &blow_wind, &dts); break;
        case 3:
            Scene::loadClothStatic(pVector, &blow_wind, &dts); break;
        case 4:
            Scene::loadClothWire(pVector, &blow_wind, &dts); break;
        case 5:
            Scene::loadHairStatic(pVector, &blow_wind, &dts); break;
        case 6:
            Scene::loadAngularSpring(pVector, &blow_wind, &dts); break;
        default:
            Scene::loadDefault(pVector, &blow_wind, &dts);
    }
    for (Particle *p: pVector) { p->reset(); }
    // Get list sizes
    m = Constraint::_constraints.size();
    n = pVector.size();
    solver = new SympleticMidpointSolver();
    state = new State(solver, n, m, pVector);

#ifdef DEBUG
    printf("init: n=%i m=%i\n", n, m);
#endif
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display(void) {
    glViewport(0, 0, win_x, win_y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

static void post_display(void) {
    // FPS
    int current_time = glutGet(GLUT_ELAPSED_TIME);
    if ((current_time / 1000.0) - last_time >= 1.0) {
        fps = n_frames / (current_time / 1000.0 - last_time);
        n_frames = 0;
        last_time += 1.0;
    }

    dts = dt_since_start * dt;
    char *buff = (char *) malloc(sizeof(char) * 1024);
    sprintf(buff, "Particletoys! - t: %.3f - fps: %.1f", dts, fps);
    glutSetWindowTitle(buff);
    free(buff);
    // Write frames if necessary.
    if (dump_frames) {

        // Write fps to file
        frametime_file << fps << "\n";

        const int FRAME_INTERVAL = 4;
        if ((frame_number % FRAME_INTERVAL) == 0) {
            const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
            const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
            unsigned char *buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
            if (!buffer) {
                exit(-1);
            }
            // glRasterPos2i(0, 0);
            glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
            static char filename[80];
            sprintf(filename, "../snapshots/img%.5i.png", frame_number / FRAME_INTERVAL);
            printf("Dumped %s.\n", filename);
            saveImageRGBA(filename, buffer, w, h);

            free(buffer);
        }
    }
    frame_number++;
    n_frames++;

    glutSwapBuffers();
}

static void draw_particles(void) {

    for (int ii = 0; ii < n; ii++) {
        pVector[ii]->draw(show_velocity, show_force);
    }
}

static void draw_forces(void) {
    for (Force *f: Force::_forces) {
        f->draw();
    }

    for (Force *f: Force::_mouse_forces) {
        f->draw();
    }
}

static void draw_constraints(void) {
    for (Constraint *c: Constraint::_constraints) {
        c->draw();
    }
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func(unsigned char key, int x, int y) {
    switch (key) {

        // Dump screen and frame times
        case 'd':
        case 'D':dump_frames = !dump_frames;
            if (dump_frames) {
                filesystem_opened++;
                static char filename[80];
                sprintf(filename, "frame_times_%.5i.txt", filesystem_opened);
                frametime_file.open(filename);
            } else {
                frametime_file.close();
            }
            break;

            // Reset simulation
        case 'r':
        case 'R':dsim = false;
            for (Particle *p: pVector) { p->reset(); }
            dt_since_start = 0;
            state->reset(pVector);
            for (Constraint *c: Constraint::_constraints) { c->eval_C(state->globals); }
            for (Force *f: Force::_forces) { f->calculate_forces(state->globals); }
            break;

            // Quit
        case 'q':
        case 'Q':free_data();
            exit(0);
            break;

            // Pause simulation
        case ' ':dsim = !dsim;
            break;

            // Toggle draw functions
        case 'v':show_velocity = !show_velocity;
            break;
        case 'b':show_force = !show_force;
            break;
        case 'w':blow_wind = !blow_wind;
            break;

            // Switch scenes
        case '1':scene_int = 1;
            init_system();
            break;
        case '2':scene_int = 2;
            init_system();
            break;
        case '3':scene_int = 3;
            init_system();
            break;
        case '4':scene_int = 4;
            init_system();
            break;
        case '5':scene_int = 5;
            init_system();
            break;
        case '9':scene_int = 6;
            init_system();
            break;
    }
}

static void mouse_func(int button, int state, int x, int y) {
    omx = mx = x;
    omx = my = y;

    if (mouse_down[button]) { mouse_release[button] = state == GLUT_UP; }
    if (mouse_down[button]) { mouse_shiftclick[button] = glutGetModifiers() == GLUT_ACTIVE_SHIFT; }
    mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func(int x, int y) {
    mx = x;
    my = y;
}

static void reshape_func(int width, int height) {
    glutSetWindow(win_id);
    glutReshapeWindow(width, height);

    win_x = width;
    win_y = height;
}

static void mouse_interact() {
    if (mouse_down[0]) {

        // Update the position of the mouse particle
        float q = mx / (float) win_x;
        float r = (win_y - my) / (float) win_y;
        q = q * 2 - 1;
        r = r * 2 - 1;
        mouse_particle->m_Position = Vec2(q, r);

        // Update the velocity of the mouse particle
        float o_q = omx / (float) win_x;
        float o_r = (win_y - omy) / (float) win_y;
        o_q = o_q * 2 - 1;
        o_r = o_r * 2 - 1;
        mouse_particle->m_Velocity = Vec2(q - o_q, r - o_r);

        // Don't add more particles if we are already dragging one or more
        if (particle_selected) { return; }

        // Bounding box of the selectable particles
        const float h = 0.03;
        Vec2 min = {q - h, r - h};
        Vec2 max = {q + h, r + h};
        int i = 0;
        for (Particle *p: pVector) {
            if (is_inside_bbox(p->m_Position, min, max)) {
                float dist = sqrt(pow(mouse_particle->m_Position[0] - p->m_Position[0], 2) +
                                  pow(mouse_particle->m_Position[1] - p->m_Position[1], 2));
                Force::_mouse_forces.push_back(new MouseSpringForce(i, mouse_particle, dist, 1.0, 0.2));
                particle_selected = true;
            }
            i++;
        }
    }

    // Remove all mouse spring forces
    if (!mouse_down[0] && particle_selected) {
        Force::_mouse_forces.clear();
        particle_selected = false;
    }
}

static void idle_func(void) {
    if (dsim) {
        for (int i = 0; i < N; i++) {
            dt_since_start++;
#ifdef DEBUG
            printf("Iteration %i\n", i);
#endif
            state->advance(dt);
        }
        state->copy_to_particles(pVector);
#ifdef STEP
        dsim = false;
#endif
        mouse_interact();
    }

    glutSetWindow(win_id);
    glutPostRedisplay();
}

static void display_func(void) {
    pre_display();

    draw_forces();
    draw_constraints();
    draw_particles();

    post_display();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window(void) {
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);

    glutInitWindowPosition(0, 0);
    glutInitWindowSize(win_x, win_y);
    win_id = glutCreateWindow("Particletoys!");

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

    pre_display();

    glutKeyboardFunc(key_func);
    glutMouseFunc(mouse_func);
    glutMotionFunc(motion_func);
    glutReshapeFunc(reshape_func);
    glutIdleFunc(idle_func);
    glutDisplayFunc(display_func);
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main(int argc, char **argv) {
    glutInit(&argc, argv);

    if (argc == 1) {
        // 144 * N * dt = 1
        N = 25;
        dt = 1 / (144.0 * N);
        d = 5.f;
        fprintf(stderr, "Using defaults : N=%d dt=%g d=%g\n",
                N, dt, d);
    } else {
        N = atoi(argv[1]);
        dt = atof(argv[2]);
        d = atof(argv[3]);
    }

    printf("\n\nHow to use this application:\n\n");
    printf("\t Toggle construction/simulation display with the spacebar key\n");
    printf("\t Dump frames by pressing the 'd' key\n");
    printf("\t Quit by pressing the 'q' key\n");

    printf("\t Show the velocities with the 'v' key\n");
    printf("\t Show the forces with the 'b' key\n");
    printf("\t Turn on/off the wind with the 'w' key\n");
    printf("\n");
    printf("\t Switch to the default scene with the '1' key\n");
    printf("\t Switch to the double circle scene with the '2' key\n");
    printf("\t Switch to the static cloth scene with the '3' key\n");
    printf("\t Switch to the sliding cloth scene with the '4' key\n");
    printf("\t Switch to the hair scene with the '5' key\n");
    printf("\t Switch to the angular spring scene with the '9' key\n");

    dsim = 0;
    dump_frames = 0;
    frame_number = 0;

    mouse_particle = new Particle(Vec2(0, 0));

    init_system();

    win_x = 1024;
    win_y = 1024;
    open_glut_window();

    glutMainLoop();

    exit(0);
}


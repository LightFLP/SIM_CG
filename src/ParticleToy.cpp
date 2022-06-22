// ParticleToy.cpp : Defines the entry point for the console application.
//
// #define DEBUG
// #define STEP
#define TARGET_FPS 30
#define TIMESTEPS_PER_FRAME 100
#define DUMP_FREQUENCY 2
#define IX(i,j) ((i)+(N_f+2)*(j)) // fluid
#include <gfx/geom3d.h>
#include <GL/glut.h>

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <cstring>
#include <algorithm>

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

Solver *solver;
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
static int solver_int = 1;

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
static bool fluid_interaction = false;
static bool collision = false;

/* fluid global variables */

/* external definitions (from solver.c) */

extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt, bool * solid);
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, bool * solid );

static int N_f;
static float dt_f, diff, visc;
static float force, source;
static int dvel;

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;
static bool * solid;
static bool removeSolid;

struct Body {
    float center_of_mass_x;
    float center_of_mass_y;
    std::vector<int> position;
};

static std::vector<Body> bodies;
static bool has_changed;



/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data(void) {
    pVector.clear();

    // fluid
    if ( u ) free ( u );
    if ( v ) free ( v );
    if ( u_prev ) free ( u_prev );
    if ( v_prev ) free ( v_prev );
    if ( dens ) free ( dens );
    if ( dens_prev ) free ( dens_prev );
    if ( solid ) free ( solid );
}

static void clear_data_fluid ( void )
{
    int i, size=(N_f+2)*(N_f+2);

    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
        solid[i] = false;
    }
}

static int allocate_data_fluid ( void )
{
    int size = (N_f+2)*(N_f+2);

    u			= (float *) malloc ( size*sizeof(float) );
    v			= (float *) malloc ( size*sizeof(float) );
    u_prev		= (float *) malloc ( size*sizeof(float) );
    v_prev		= (float *) malloc ( size*sizeof(float) );
    dens		= (float *) malloc ( size*sizeof(float) );
    dens_prev	= (float *) malloc ( size*sizeof(float) );
    solid		= (bool *)  malloc ( size*sizeof(bool) );

    if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev ) {
        fprintf ( stderr, "cannot allocate data\n" );
        return ( 0 );
    }

    return ( 1 );
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
            Scene::loadFluid(pVector, &blow_wind, &collision, &dts, N_f, u, v); break;
        case 2:
            Scene::loadDefault(pVector, &blow_wind, &collision, &dts, N_f, u, v, dens); break;
        case 3:
            Scene::loadDoubleCircle(pVector, &blow_wind,&collision, &dts, N_f, u, v); break;
        case 4:
            Scene::loadClothStatic(pVector, &blow_wind,&collision, &dts, N_f, u, v, dens); break;
        case 5:
            Scene::loadClothWire(pVector, &blow_wind,&collision, &dts, N_f, u, v, dens); break;
        case 6:
            Scene::loadHairStatic(pVector, &blow_wind,&collision, &dts, N_f, u, v, dens); break;
        case 7:
            Scene::loadAngularSpring(pVector, &blow_wind,&collision, &dts, N_f, u, v); break;
        default:
            Scene::loadDefault(pVector, &blow_wind,&collision, &dts, N_f, u, v, dens);
    }
    for (Particle *p: pVector) { p->reset(); }
    // Get list sizes
    m = Constraint::_constraints.size();
    n = pVector.size();

    switch (solver_int) {
        case 1:
            solver = new EulerSolver(); break;
        case 2:
            solver = new SympleticEulerSolver(); break;
        case 3:
            solver = new MidpointSolver(); break;
        case 4:
            solver = new SympleticMidpointSolver(); break;
       case 5:
           solver = new RK4Solver(); break;
        default:
            solver = new EulerSolver();
    }
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
    gluOrtho2D(-1.0, 1.0, -1.0, 1.0); //fluid different
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

        const int FRAME_INTERVAL = DUMP_FREQUENCY-1;
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

static void draw_velocity_fluid ( void )
{
    int i, j;
    float x, y, h;

    h = 1.0f/N_f;

    glColor3f ( 1.0f, 1.0f, 1.0f );
    glLineWidth ( 1.0f );

    glBegin ( GL_LINES );

    for ( i=1 ; i<=N_f ; i++ ) {
        x = (i-0.5f)*h;
        x = x*2 -1;

        for ( j=1 ; j<=N_f ; j++ ) {
            y = (j-0.5f)*h;
            y = y*2 -1;

            glVertex2f ( x, y );
            glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
        }
    }

    glEnd ();
}

// https://www.shadertoy.com/view/ll2GD3
static void set_color_fluid(float dens)
{
    dens = 1-dens;
    dens *= .85f;
    const float magic = 6.28318f;
    glColor3f(
            .5f + .5f * cosf(magic * (dens + .00f)),
            .5f + .5f * cosf(magic * (dens + .33f)),
            .5f + .5f * cosf(magic * (dens + .67f))
    );
}

static void draw_density_fluid ( void )
{
    int i, j;
    float x, y, h, d00, d01, d10, d11;

    h = 1.0f/N_f;

    glBegin ( GL_QUADS );

    for ( i=0 ; i<=N_f ; i++ ) {
        x = i*h;
        x = x*2 -1;
        for ( j=0 ; j<=N_f ; j++ ) {
            y = j*h;
            y = y*2 -1;

            d00 = dens[IX(i,j)];
            d01 = dens[IX(i,j+1)];
            d10 = dens[IX(i+1,j)];
            d11 = dens[IX(i+1,j+1)];

            set_color_fluid ( d00 ); glVertex2f ( x, y );
            set_color_fluid ( d10 ); glVertex2f ( x+h*2, y );
            set_color_fluid ( d11 ); glVertex2f ( x+h*2, y+h*2 );
            set_color_fluid ( d01 ); glVertex2f ( x, y+h*2 );
        }
    }



    for ( i=1 ; i<N_f+2 ; i++ ) {
        x = i*h;
        x = x*2 -1;
        for ( j=1 ; j<N_f+2 ; j++ ) {
            y = j*h;
            y = y*2 -1;
            if (solid[IX(i,j)]) {
                glColor3f ( 0.0f, 0.0f, 0.0f );
                glVertex2f ( x, y );
                glVertex2f ( x+h*2, y );
                glVertex2f ( x+h*2, y+h*2 );
                glVertex2f ( x, y+h*2 );
            }
        }
    }
    glEnd ();
}

// Depth first search the cells that connect horizontally or vertically
static int has_neighbour ( std::vector<int> *objects, bool * solid, int i, int j )
{
    if (solid[IX(i, j)] && solid[IX(i + 1, j)]) {
        has_neighbour(objects, solid, i + 1, j);
        objects->push_back(IX(i + 1, j));
    }
    if (solid[IX(i, j)] && solid[IX(i, j + 1)]) {
        has_neighbour(objects, solid, i, j + 1);
        objects->push_back(IX(i, j + 1));
    }
    if (solid[IX(i, j)]) {
        objects->push_back(IX(i, j));
//        printf("%i, %i\n", i, j);
    }
    return -1;
}

// checks if there are any objects in the scene
// it searches the grid using a DFS and groups the cells
static void create_objects ( bool * solid )
{
    int i, j;

    // loop over all cells in the grid and add the cell numbers that have neighbours to a giant array
    std::vector<int> int_objects;
    for ( i=1 ; i<N+2 ; i++ ) {
        for ( j=1 ; j<N+2 ; j++ ) {
            if (std::find(int_objects.begin(), int_objects.end(), IX(i, j)) == int_objects.end()) {
                has_neighbour( &int_objects, solid, i, j ); // DFS
                int_objects.push_back(-1); // spacer between solids
            }
        }
    }

    // thin out the array by removing the spacers (-1) and grouping them into individual arrays
    Body body = Body();
    for (i = 0; i < int_objects.size(); i++) {
        while(int_objects[i] != -1) {
            body.position.push_back(int_objects[i]);
            i++;
        }
        if (!body.position.empty() && body.position[0] != -1) {
            bodies.push_back(body);
            body = Body();
        }
    }

    // remove duplicates as the DFS isn't that smart
    for (Body &o : bodies) {
        std::sort(o.position.begin(), o.position.end());
        o.position.erase(std::unique(o.position.begin(), o.position.end()), o.position.end());
    }

    // calculate center of mass
    // all masses are the same so just get the center cell
    float centerX, centerY;
    for (Body o : bodies) {
        centerX = centerY = 0;
        for (int q : o.position) {
            centerX += q % (N+2);
            centerY += q / (N+2);
        }
        o.center_of_mass_x = centerX / o.position.size();
        o.center_of_mass_y = centerY / o.position.size();
    }

    // print the number of solid objects currently in the scene
//    printf("solid bodies=%i\n", bodies.size());

//    for (Body o : bodies) {
//        for (int q : o.position) {
//            printf("%i ", q);
//        }
//        printf("\n");
//    }
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
        case 'D':
            dump_frames = !dump_frames;
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
        case 'R':
            dsim = false;
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
        case ' ':
            dsim = !dsim; break;

            // Toggle fluid controls
        case  'f':
        case  'F':
            fluid_interaction = !fluid_interaction;

            if(fluid_interaction){
                printf ( "\t Fluid controls toggled ON\n" );
            }else{
                printf ( "\t Fluid controls toggled OFF\n" );
            }
            break;

            // Toggle solid bodies add and remove
        case 'p':
        case 'P':
            if(fluid_interaction){
                removeSolid = !removeSolid;
            }
            break;

            // Toggle draw functions
        case 'v':
        case 'V':
            if(fluid_interaction){
                dvel = !dvel;
            }else{
                show_velocity = !show_velocity;
            }
            break;

        case 'b':
        case 'B':
            show_force = !show_force; break;
        case 'w':
        case 'W':
            blow_wind = !blow_wind; break;
        case 'c':
        case 'C':
            if(!fluid_interaction){
                collision = !collision;
            }else{
                clear_data_fluid();
            }
            break;

            // Switch solver
        case '1':
            solver_int = 1;
            init_system();
            break;
        case '2':
            solver_int = 2;
            init_system();
            break;
        case '3':
            solver_int = 3;
            init_system();
            break;
        case '4':
            solver_int = 4;
            init_system();
            break;
       case '5':
           solver_int = 5;
           init_system();
           break;
//        case '6':
//            solver_int = 6;
//            init_system();
//            break;

            // Switch scene
        case '.':
            scene_int += scene_int == 6 ? 0 : 1; // NB keep this in sync with the number of scenes
            init_system();
            break;

        case ',':
            scene_int -= scene_int == 1 ? 0 : 1;
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

//fluid
static void get_from_UI_fluid ( float * d, float * u, float * v, bool * solid )
{
    int i, j, size = (N_f+2)*(N_f+2);

    for ( i=0 ; i<size ; i++ ) {
        u[i] = v[i] = d[i] = 0.0f;
    }

    if(fluid_interaction){

        if ( !mouse_down[0] && !mouse_down[1] && !mouse_down[2] ) return;

        i = (int)((       mx /(float)win_x)*N_f+1);
        j = (int)(((win_y-my)/(float)win_y)*N_f+1);

        if ( i<1 || i>N_f || j<1 || j>N_f ) return;

        if ( mouse_down[0] ) {
            u[IX(i,j)] = force * (mx-omx);
            v[IX(i,j)] = force * (omy-my);
        }

        if ( mouse_down[1] ) {
            if (solid[IX(i,j)] != removeSolid) {
                solid[IX(i,j)] = removeSolid;
                has_changed = true;
            }
        }

        if ( mouse_down[2] ) {
            d[IX(i,j)] = source;
        }

        omx = mx;
        omy = my;

    }

    return;
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

        // fluid
        get_from_UI_fluid ( dens_prev, u_prev, v_prev, solid );

        if (has_changed) {
            create_objects ( solid );
            has_changed = false;
        }

        vel_step ( N_f, u, v, u_prev, v_prev, visc, dt_f, solid );
        dens_step ( N_f, dens, dens_prev, u, v, diff, dt_f, solid );

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

    // fluid
    if ( dvel ) draw_velocity_fluid();
    else		draw_density_fluid();

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
        N = TIMESTEPS_PER_FRAME; 
        dt = 1.0 / (DUMP_FREQUENCY * TARGET_FPS * N);
        d = 5.f;

        //fluid
        N_f = 64;
        dt_f = 0.1f;
        diff = 0.0f;
        visc = 0.0f;
        force = 5.0f;
        source = 100.0f;

        fprintf(stderr, "Using defaults for particle toy : N=%d dt=%g d=%g\n",
                N, dt, d);
        fprintf ( stderr, "Using defaults for fluid : N_f=%d dt_f=%g diff=%g visc=%g force = %g source=%g\n",
                  N_f, dt_f, diff, visc, force, source );
    } else {
        N = atoi(argv[1]);
        dt = atof(argv[2]);
        d = atof(argv[3]);

        N_f = atoi(argv[1]);
        dt_f = atof(argv[2]);
        diff = atof(argv[3]);
        visc = atof(argv[4]);
        force = atof(argv[5]);
        source = atof(argv[6]);
    }

    printf("\n\nHow to use this application:\n\n");
    printf("\t Toggle construction/simulation display with the spacebar key\n");
    printf("\t Dump frames by pressing the 'd' key\n");
    printf("\t Quit by pressing the 'q' key\n");

    printf("\t Show the velocities with the 'v' key\n");
    printf("\t Show the forces with the 'b' key\n");
    printf("\t Turn on/off the wind with the 'w' key\n");
    printf("\t Turn on/off the wall collision with the 'c' key\n");
    printf("\n");
    printf("\t Switch between scenes using the '<' and '>' keys\n");
    printf("\n");
    printf("\t Switch the solver to Euler with the '1' key\n");
    printf("\t Switch the solver to Sympletic Euler with the '2' key\n");
    printf("\t Switch the solver to Midpoint with the '3' key\n");
    printf("\t Switch the solver to Sympletic Midpoint with the '4' key\n");
    printf("\t Switch the solver to Runge-Kutta with the '5' key\n");
    printf("\t Switch the solver to Simpletic Runge-Kutta with the '6' key\n");


    printf ( "\n\nToggle on/off the fluid interaction controls with the 'f' key\n\n" );

    printf ( "\n\nFluid interaction:\n\n" );
    printf ( "\t Add densities with the right mouse button\n" );
    printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
    printf ( "\t Add or remove solid cells with the middle mouse button\n \t Switch between adding/removing the solid cells by pressing the 'p' key \n" );
    printf ( "\t Toggle density/velocity display with the 'v' key\n" );
    printf ( "\t Clear the simulation by pressing the 'c' key\n" );


    dsim = 0;
    dump_frames = 0;
    frame_number = 0;

    //fluid
    dvel = 0;

    if ( !allocate_data_fluid () ) exit ( 1 );
    clear_data_fluid ();

    // check this once before running
//    create_objects ( solid );

    mouse_particle = new Particle(Vec2(0, 0));

    init_system();

    win_x = 1024;
    win_y = 1024;
    open_glut_window();

    glutMainLoop();

    exit(0);
}

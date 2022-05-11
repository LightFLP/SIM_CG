// ParticleToy.cpp : Defines the entry point for the console application.
//

#include <gfx/geom3d.h>

#include "Particle.h"
#include "SpringForce.h"
#include "RodConstraint.h"
#include "CircularWireConstraint.h"
#include "imageio.h"
#include "EulerSolver.h"
#include "ConstantForce.h"
#include "RK4Solver.h"
#include "DragForce"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>


/* macros */

Solver* solver = new RK4Solver();
std::vector<Force*> forces = std::vector<Force*>();
std::vector<Constraint*> constraints = std::vector<Constraint*>();

/* global variables */

static int N;
static double dt, d;
static int dsim;
static int dump_frames;
static int frame_number;

// static Particle *pList;
static std::vector<Particle*> pVector;
static Particle* mouseParticle;
static int was_selected;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

// booleans
static bool show_velocity = false;
static bool show_force = false;

static SpringForce * delete_this_dummy_spring = NULL;
static RodConstraint * delete_this_dummy_rod = NULL;
static CircularWireConstraint * delete_this_dummy_wire = NULL;


/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
	pVector.clear();
	if (delete_this_dummy_rod) {
		delete delete_this_dummy_rod;
		delete_this_dummy_rod = NULL;
	}
	if (delete_this_dummy_spring) {
		delete delete_this_dummy_spring;
		delete_this_dummy_spring = NULL;
	}
	if (delete_this_dummy_wire) {
		delete delete_this_dummy_wire;
		delete_this_dummy_wire = NULL;
	}
}

static void clear_data ( void )
{
	int ii, size = pVector.size();

	for(ii=0; ii<size; ii++){
		pVector[ii]->reset();
	}
}

// \emph{q} positions
static double* global_q;

// \emph{\dot{q}} velocities
static double* global_qdot;

// \emph{Q} forces
static double* global_forces;

static double* global_C;
static double* global_Cdot;

// RHS for solving the constraints
static double* global_RHS; //RHS = -Jdot*qdot - JQ - ks*C -kd*Cdot.
static double  ks =     0; //stiffness for the RHS
static double  kd =     0; //damping   for the RHS

static implicitMatrixWithTrans* J;
static implicitMatrixWithTrans* Jdot;
static implicitJWJt* JWJt;

// number of constraints
static int m;
// number of particles
static int n;


static void init_system(void)
{
#if DEBUG
	implicitMatrixWithTrans* test = new implicitMatrixWithTrans(3, 3);
	double* test_vec = (double*) malloc(sizeof(double) * 3);
	test_vec[0] = 1;
	test_vec[1] = 1;
	test_vec[2] = 1;
	MatrixBlock* mb = new MatrixBlock(0, 1, 2, 2);
	mb->data[0] = 2;
	mb->data[1] = 1;
	mb->data[2] = 2;
	mb->data[3] = 0;
	


	test->blocks.push_back(mb);

	double* test_output = (double*) malloc(sizeof(double) * 3);
	test->matVecMult(test_vec, test_output);
	for (int i = 0; i < 3; i++){
		printf("%i: %f\n", i, test_output[i]);
	}
	test->matTransVecMult(test_vec, test_output);
	for (int i = 0; i < 3; i++){
		printf("%i: %f\n", i, test_output[i]);
	}
	exit(0);
#endif

	const double dist = 0.2;
	const Vec2 center(0.0, 0.0);
	const Vec2 offset(dist, 0.0);

	// Create three particles, attach them to each other, then add a
	// circular wire constraint to the first.

	pVector.push_back(new Particle(center + offset));
	pVector.push_back(new Particle(center + offset + offset ));
	pVector.push_back(new Particle(center + offset + offset + offset ));
	// pVector.push_back(new Particle(center + offset + offset + offset + offset));
	
	forces.push_back(new ConstantForce(Vec2(0, -9.81))); // Graivty
	forces.push_back(new DragForce(0.05)); // drag

	bool skip = false;
	for (Particle* p : pVector){
		if (skip){
			skip = false;
			continue;
		}
		forces[0]->register_particle(p); //Gravity
		forces[1]->register_particle(p);
	}


	forces.push_back(new SpringForce(pVector[0], pVector[1], dist, 500.0, 0.2));
	
	constraints.push_back(new CircularWireConstraint(pVector[0], 0, 0, Vec2(0, 0), dist));
	constraints.push_back(new CircularWireConstraint(pVector[1], 1, 1, 3*offset, dist));
	constraints.push_back(new RodConstraint(pVector[1], pVector[2], 1, 2, 2, dist));

	J = new implicitMatrixWithTrans(constraints.size(), pVector.size() * 2);
	Jdot = new implicitMatrixWithTrans(constraints.size(), pVector.size() * 2);
	JWJt = new implicitJWJt(J);
	JWJt->W = (double*) malloc(sizeof(double) * pVector.size() * 2);
	for (int i = 0; i < pVector.size(); i++){
		JWJt->W[2*i+0] = 1/pVector[i]->m_Mass;
		JWJt->W[2*i+1] = 1/pVector[i]->m_Mass;
	}

	for (Constraint *c : constraints){
		// we need to register the corresponding matrix blocks with the linear solver
		for (MatrixBlock* mb : c->matrix_blocks_J){
			J->blocks.push_back(mb);
		}
		for (MatrixBlock* mb : c->matrix_blocks_Jdot){
			Jdot->blocks.push_back(mb);
		}
	}
	m = constraints.size();
	n = pVector.size();
	
	printf("init: n=%i m=%i\n", n, m);
	printf("init: |J blocks| = %i\n", J->blocks.size());

	// Definitely know how many particles there are going to be
	global_q = (double*) malloc(sizeof(double) * n * 2);
	global_qdot = (double*) malloc(sizeof(double) * n * 2);
	global_forces = (double*) malloc(sizeof(double) * n * 2);

	global_RHS = (double*) malloc(sizeof(double) * m);
	global_C = (double*) malloc(sizeof(double) * m);
	global_Cdot = (double*) malloc(sizeof(double) * m);
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( -1.0, 1.0, -1.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 4;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
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
	
	glutSwapBuffers ();
}

static void draw_particles ( void )
{

	for(int ii=0; ii< n; ii++)
	{
		pVector[ii]->draw(show_velocity, show_force);
	}
}

static void draw_forces ( void )
{
	for (Force* f : forces){
		f->draw();
	}
}

static void draw_constraints ( void )
{
	for (Constraint* c : constraints){
		c->draw();
	}
}

/*
----------------------------------------------------------------------
relates mouse movements to particle toy construction
----------------------------------------------------------------------
*/

static void get_from_UI ()
{
	int i, j;
	// int size, flag;
	int hi, hj;
	// double x, y;
	if ( !mouse_down[0] && !mouse_down[2] && !mouse_release[0] 
	&& !mouse_shiftclick[0] && !mouse_shiftclick[2] ) return;

	i = (int)((       mx /(double)win_x)*N);
	j = (int)(((win_y-my)/(double)win_y)*N);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {

	}

	if ( mouse_down[2] ) {

	}

	hi = (int)((       hmx /(double)win_x)*N);
	hj = (int)(((win_y-hmy)/(double)win_y)*N);

	if( mouse_release[0] ) {
		
	}

	omx = mx;
	omy = my;
}

static void remap_GUI()
{
	int ii, size = pVector.size();
	for(ii=0; ii<size; ii++)
	{
		pVector[ii]->m_Position[0] = pVector[ii]->m_ConstructPos[0];
		pVector[ii]->m_Position[1] = pVector[ii]->m_ConstructPos[1];
	}
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
	case 'c':
	case 'C':
		clear_data ();
		break;

	case 'd':
	case 'D':
		dump_frames = !dump_frames;
		break;

	case 'q':
	case 'Q':
		free_data ();
		exit ( 0 );
		break;

	case ' ':
		dsim = !dsim;
		if (dsim) for (Particle *p : pVector) p->reset();
		break;

    case 'v':
        show_velocity = !show_velocity;
        break;

    case 'b':
        show_force = !show_force;
        break;
    }
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

bool check_bit(int number, int pos)
{
    return (number >> pos) & 1U;
}

void set_bit(int& number, int pos)
{
    number |= 1UL << pos;
}

void clear_bit(int& number, int pos)
{
    number &= ~(1UL << pos);
}

static void mouse_interact ()
{
    if (mouse_down[0]) {

        // Update the position of the mouse particle
        float q = mx / (float)win_x;
        float r = (win_y - my) / (float)win_y;
        q = q * 2 - 1;
        r = r * 2 - 1;
        mouseParticle->m_Position = Vec2(q, r);

        // Update the velocity of the mouse particle
        float o_q = omx / (float)win_x;
        float o_r = (win_y - omy) / (float)win_y;
        o_q = o_q * 2 - 1;
        o_r = o_r * 2 - 1;
        mouseParticle->m_Velocity = Vec2(q - o_q, r - o_r);

        // Don't add more particles if we are already dragging one
        if (was_selected > 0) return;

        // Bounding box of the selectable particles
        const float h = 0.03;
        Vec2 min = {q - h, r - h};
        Vec2 max = {q + h, r + h};

        int idx = 0;
        for (Particle* p : pVector) {
            if (!check_bit(was_selected, idx) && is_inside_bbox(p->m_Position, min, max)) {
                float dist = sqrt(pow(mouseParticle->m_Position[0] - p->m_Position[0], 2) + pow(mouseParticle->m_Position[1] - p->m_Position[1], 2));
                forces.push_back(new SpringForce(p, mouseParticle, dist, 50.0, 0.2));
                set_bit(was_selected, idx);
            }
            idx++;
        }
    }

    // Remove all mouse spring forces from the list
    if (!mouse_down[0] && was_selected > 0) {
        int idx = 0;
        while (was_selected > 0) {
            if (check_bit(was_selected, idx)) forces.pop_back();
            clear_bit(was_selected, idx);
            idx++;
        }
    }
}

static bool populate_globals_verbose = false;
static void populate_globals(){
	Particle* p;
	for (int i = 0; i < n; i++){
		p = pVector[i];

		global_q[2*i  ] = p->m_Position[0];
		global_q[2*i+1] = p->m_Position[1];

		global_qdot[2*i  ] = p->m_Velocity[0];
		global_qdot[2*i+1] = p->m_Velocity[1];

		//Since Q only used in JWQ, we can smuggle W in there as well
		global_forces[2*i  ] = p->m_ForceAccum[0]/p->m_Mass;
		global_forces[2*i+1] = p->m_ForceAccum[1]/p->m_Mass;
	}

	Constraint* c;
	std::memset(global_C, 0.0, m);
	std::memset(global_Cdot, 0.0, m);
	for (int i = 0; i < m; i++){
		c = constraints[i];
		global_C[i] += c->eval_C();
		global_Cdot[i] += c->eval_Cdot();
		if (populate_globals_verbose) printf("C[%i] = %.3f C.[%i] = %.3f\n", i, global_C[i], i, global_Cdot[i]);
	}


	// setup global_RHS
	double*  Jq = (double*) malloc(sizeof(double) * m);
	double* JWQ = (double*) malloc(sizeof(double) * m);
	double* ksC = (double*) malloc(sizeof(double) * m);
	double* kdC = (double*) malloc(sizeof(double) * m);

	// Jq = Jdot * Qdot
	if (populate_globals_verbose) {
		printf("\n< Jq <- Jdot*qdot>\n");
		printf("\nglobal_qdot:\n");
		for (int i = 0; i < n; i++){
			printf(" q.[%i] = (%.3f, %.3f)\n", i, global_qdot[2*i], global_qdot[2*i+1]);
		}
		printf("\n</Jq <- Jdot*qdot>\n");
	}
	Jdot->matVecMult(global_qdot, Jq, populate_globals_verbose);

	J->matVecMult(global_forces, JWQ, populate_globals_verbose);
	// JWQ = J * (W*Q)
	if (populate_globals_verbose) {
		printf("\n< JWQ <- J*WQ>\n");
		printf("\nglobal_forces:\n");
		for (int i = 0; i < n; i++){
			printf(" WQ[%i] = (%.3f, %.3f)\n", i, global_forces[2*i], global_forces[2*i+1]);
		}
		printf("\nresult:\n");
		for (int i = 0; i < m; i++){
			printf("JWQ[%i] = %.3f\n", i, JWQ[i]);
		}
		printf("\n</JWQ <- J*WQ>\n");
	}

	// ksC = C
	// ksC *= ks
	vecAssign(m, ksC, global_C);
	vecTimesScalar(m, ksC, ks);

	// kdC = Cdot
	// kdC *= kd
	vecAssign(m, kdC, global_Cdot);
	vecTimesScalar(m, kdC, kd);

	//Assemble RHS
	
	memset(global_RHS, 0.0, sizeof(double) * m);

	vecDiffEqual(m, global_RHS, Jq); // RHS = -Jq
	vecDiffEqual(m, global_RHS, JWQ); // RHS = -Jq - JWQ
	vecDiffEqual(m, global_RHS, ksC); // RHS = -Jq - JWQ - ksC
	vecDiffEqual(m, global_RHS, kdC); // RHS = -JQ - JWQ - ksC - kdC
	// vecTimesScalar(m, global_RHS, -1); //RHS = -Jq - JWQ - ksC - kdC

	if (populate_globals_verbose) {
		printf("\n< global_RHS>\n");
		for(int i = 0; i < m; i++){
			printf(" b[%i] = %.3f\n", i, global_RHS[i]);
		}
		printf("</global_RHS>\n");
	}

}

static void idle_func ( void )
{
	if ( dsim ){
		
		for (Particle* p : pVector) p->m_ForceAccum = Vec2(0, 0); // Clear forces
		for (Force *f : forces) f->calculate_forces(); // Calculate all forces
		//Constraint handling
		for (Constraint *c : constraints){
			c->eval_J();
			c->eval_Jdot();
		}
		populate_globals();
		
		double* lambda = (double*) malloc(sizeof(double) * m);



		int steps = 0;


		ConjGrad(m, JWJt, lambda, global_RHS, 1e-32, &steps);

		// printf("\n< lambda>\n");
		// for (int i = 0; i < m; i++){
		// 	printf(" lambda[%i] = %.3f", i, lambda[i]);
		// }
		// printf("\n</lambda>\n");
		double* Qhat = (double*) malloc(sizeof(double) * 2 * n);
		J->matTransVecMult(lambda, Qhat);
		// for (int i = 0; i < n; i++){
		// 	printf("Qhat[%i] = (%.3f, %.3f)\n", i, Qhat[2*i], Qhat[2*i+1]);
		// }
		Particle* p;
		for (int i = 0; i < n; i++){
			p = pVector[i];
			p->m_ForceAccum[0] += Qhat[2*i];
			p->m_ForceAccum[1] += Qhat[2*i+1];
		}

		solver->simulation_step( pVector, dt );

        mouse_interact();
	}else{
		get_from_UI();
		remap_GUI();
	}

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

	draw_forces();
	draw_constraints();
	draw_particles();

	post_display ();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Particletoys!" );

	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc == 1 ) {
		N = 64;
		dt = 1e-4f;
		d = 5.f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g d=%g\n",
			N, dt, d );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		d = atof(argv[3]);
	}

	printf ( "\n\nHow to use this application:\n\n" );
	printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
	printf ( "\t Dump frames by pressing the 'd' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	printf ( "\t Show the velocities with the 'v' key\n" );
	printf ( "\t Show the forces with the 'b' key\n" );

	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

    mouseParticle = new Particle(Vec2(0,0));
	
	init_system();
	
	win_x = 1024;
	win_y = 1024;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}


#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float * tmp=x0;x0=x;x=tmp;}
#define FOR_EACH_CELL for ( i=1 ; i<=N ; i++ ) { for ( j=1 ; j<=N ; j++ ) {
#define END_FOR }}

#include <math.h>
#include <algorithm>
#include <iostream>



void add_source ( int N, float * x, float * s, float dt )
{
    int i, size=(N+2)*(N+2);
    for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

void set_bnd ( int N, int b, float * x, bool * solid )
{
    int i;

    for ( i=1 ; i<=N ; i++ ) {
        x[IX(0  ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
        x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
        x[IX(i,0  )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
        x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
    }
    x[IX(0  ,0  )] = 0.5f*(x[IX(1,0  )]+x[IX(0  ,1)]);
    x[IX(0  ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0  ,N)]);
    x[IX(N+1,0  )] = 0.5f*(x[IX(N,0  )]+x[IX(N+1,1)]);
    x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);

    //apply boundaries to any placed solid objects
    int j;
    FOR_EACH_CELL
            if (0 < i && i < N+1 && 0 < j && j < N+1) {
                //set density of every solid to 0.0 by default
                if (solid[IX(i,j)]){
                    x[IX(i,j)] = 0.0;
                }

                //set density and velocity of solid cells at the boundary of the solid object, similar to boundary of the window
                if (solid[IX(i, j)] && !solid[IX(i+1, j)]) {
                    x[IX(i  ,j  )] 	= b==1 ? -x[IX(i+1,j  )] : x[IX(i+1,j  )];
                }
                if (solid[IX(i, j)] && !solid[IX(i-1, j)]) {
                    x[IX(i  ,j  )] 	= b==1 ? -x[IX(i-1,j  )] : x[IX(i-1,j  )];
                }
                if (solid[IX(i, j)] && !solid[IX(i, j+1)]) {
                    x[IX(i  ,j  )] 	= b==2 ? -x[IX(i,j+1  )] : x[IX(i,j+1  )];
                }
                if (solid[IX(i, j)] && !solid[IX(i, j-1)]) {
                    x[IX(i  ,j  )] 	= b==2 ? -x[IX(i,j-1  )] : x[IX(i,j-1  )];
                }

                //sets density of corner to resolve issue with drawing density
                if (solid[IX(i, j)] && solid[IX(i-1, j)] && solid[IX(i, j-1)] && !solid[IX(i-1, j-1)]) {
                    x[IX(i  ,j)] = x[IX(i-1,j-1)];
                }
            }
    END_FOR
}

void lin_solve ( int N, int b, float * x, float * x0, float a, float c , bool * solid)
{
    int i, j, k;

    for ( k=0 ; k<20 ; k++ ) {
        FOR_EACH_CELL
                x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+x[IX(i,j-1)]+x[IX(i,j+1)]))/c;
        END_FOR
        set_bnd ( N, b, x, solid );
    }
}

void diffuse ( int N, int b, float * x, float * x0, float diff, float dt, bool * solid )
{
    float a=dt*diff*N*N;
    lin_solve ( N, b, x, x0, a, 1+4*a, solid );
}

void advect ( int N, int b, float * d, float * d0, float * u, float * v, float dt, bool * solid )
{
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;

    dt0 = dt*N;
    FOR_EACH_CELL
            x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
            if (x<0.5f) x=0.5f;
            if (x>N+0.5f) x=N+0.5f;
            i0=(int)x; i1=i0+1;
            if (y<0.5f) y=0.5f;
            if (y>N+0.5f) y=N+0.5f;
            j0=(int)y; j1=j0+1;
            s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
            d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
                         s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
    END_FOR
    set_bnd ( N, b, d, solid );
}

void project ( int N, float * u, float * v, float * p, float * div, bool * solid )
{
    int i, j;

    FOR_EACH_CELL
            div[IX(i,j)] = -0.5f*(u[IX(i+1,j)]-u[IX(i-1,j)]+v[IX(i,j+1)]-v[IX(i,j-1)])/N;
            p[IX(i,j)] = 0;
    END_FOR
    set_bnd ( N, 0, div, solid ); set_bnd ( N, 0, p, solid );

    lin_solve ( N, 0, p, div, 1, 4, solid );

    FOR_EACH_CELL
            u[IX(i,j)] -= 0.5f*N*(p[IX(i+1,j)]-p[IX(i-1,j)]);
            v[IX(i,j)] -= 0.5f*N*(p[IX(i,j+1)]-p[IX(i,j-1)]);
    END_FOR
    set_bnd ( N, 1, u, solid ); set_bnd ( N, 2, v, solid );
}

void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt, bool * solid )
{
    add_source ( N, x, x0, dt );
    SWAP ( x0, x ); diffuse ( N, 0, x, x0, diff, dt, solid );
    SWAP ( x0, x ); advect ( N, 0, x, x0, u, v, dt, solid );
}

float curl(int N, int i, int j, float* u, float* v) {
    float dy = (u[IX(i, j + 1)] - u[IX(i, j -1)]) * 0.5f;
    float dx = (v[IX(i + 1, j)] - v[IX(i - 1, j)]) * 0.5f;
    return dy - dx;
}

void vorticity_conf(float* u0, float* v0, int N, float* u, float* v) {
    float dx, dy, len, V;
    float* curl_arr = new float[(N+2)*(N+2)]();
    int i, j;

    for (i = 2; i < N; ++i) {
        for (j = 2; j < N; ++j) {
            curl_arr[IX(i, j)] = abs(curl(N, i, j, u, v));
        }
    }

    for (i = 2; i < N; ++i) {
        for (j = 2; j < N; ++j) {
            dx = (float)(curl_arr[IX(i + 1, j)] - curl_arr[IX(i - 1, j)]);
            dy = (float)(curl_arr[IX(i, j + 1)] - curl_arr[IX(i, j - 1)]);

            len = (float)sqrt(dx * dx + dy * dy) + 0.00001f;
            //std::cout<<len;

            dx /= len;
            dy /= len;

            V = curl(N, i, j, u, v);

            u0[IX(i,j)] = dy * -V;
            v0[IX(i,j)] = dx * V;
        }
    }
}

void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt, bool * solid )
{
    add_source ( N, u, u0, dt ); add_source ( N, v, v0, dt );
    vorticity_conf(u0, v0, N, u, v);
    add_source(N, u, u0, dt);
    add_source(N, v, v0, dt);

    SWAP ( u0, u ); diffuse ( N, 1, u, u0, visc, dt, solid);
    SWAP ( v0, v ); diffuse ( N, 2, v, v0, visc, dt, solid );
    project ( N, u, v, u0, v0, solid );
    SWAP ( u0, u ); SWAP ( v0, v );
    advect ( N, 1, u, u0, u0, v0, dt, solid ); advect ( N, 2, v, v0, u0, v0, dt, solid );
    project ( N, u, v, u0, v0, solid );
}
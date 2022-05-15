#pragma once

#include <gfx/Vec2.h>
#include <cstring>

class GlobalVars{
public:
    int n; // particles
    double* data;

    // state vectors of size 2*n
    double* x;
    double* v;
    double* Q;
    double* W;

    int m; // constraints
    // state vectors of size m
    double* C;
    double* Cdot;

    Vec2 get_pos(int p_index);

    Vec2 get_vel(int p_index);

    void set_pointers();

    GlobalVars(int _n, int _m);

    GlobalVars(const GlobalVars* other);

    ~GlobalVars(){
        printf("Globals data got freed.\n");
        free(data);
    }
};
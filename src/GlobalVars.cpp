#include "GlobalVars.h"

#define DATA_SIZE(n, m) 8 * n + 2 * m

Vec2 GlobalVars::get_pos(int p_index) {
    return Vec2(x[2 * p_index], x[2 * p_index + 1]);
}

Vec2 GlobalVars::get_vel(int p_index) {
    return Vec2(v[2 * p_index], v[2 * p_index + 1]);
}


GlobalVars::GlobalVars(int _n, int _m) {
    n = _n;
    m = _m;

    set_pointers();
}

GlobalVars::GlobalVars(const GlobalVars *other) {
    n = other->n;
    m = other->m;

    set_pointers();
    memcpy(other->data, data, sizeof(double) * DATA_SIZE(n, m));
}

void GlobalVars::set_pointers() {
    // This is cool
    size = DATA_SIZE(n, m);
    data = (double *) malloc(sizeof(double) * size);
    std::memset(data, 0.0, sizeof(double) * size);
    x = data;
    v = x + 2 * n;
    Q = v + 2 * n;
    W = Q + 2 * n;

    C = W + 2 * n;
    Cdot = C + m;
}
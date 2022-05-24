#include <cstdio>

#include "GlobalVars.h"
#include "util.h"
#include <string>

void Util::PrintGlobals(GlobalVars *globals, std::string name) {
    printf("%10s size (n=%i, m=%i). \n\tParticle properties:\n", name.c_str(), globals->n, globals->m);
    for (int i = 0; i < globals->n; i++) {
        int x = 2 * i;
        int y = 2 * i + 1;
        printf("\tx: (%10.3f, %10.3f) v: (%10.3f, %10.3f) Q: (%10.3f, %10.3f) W: (%10.3f, %10.3f)\n", globals->x[x],
               globals->x[y], globals->v[x], globals->v[y], globals->Q[x], globals->Q[y], globals->W[x], globals->W[y]);
    }
    printf("\n\tConstraint properties:\n");
    for (int i = 0; i < globals->m; i++) {
        printf("\tC: %10.3f Cdot: %10.3f\n", globals->C[i], globals->Cdot[i]);
    }
    printf("\n\n");
} 
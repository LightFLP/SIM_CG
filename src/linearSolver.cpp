#include "linearSolver.h"

// vector helper functions

void implicitJWJt::to_matrix(double *data) {
    double *Jdata = (double *) malloc(sizeof(double) * J->nrows * J->ncols);
    std::memset(Jdata, 0.0, sizeof(double) * J->nrows * J->ncols);
    J->to_matrix(Jdata);
    int i = 0;
    for (int r = 0; r < nrows; r++) {
        for (int c = 0; c < ncols; c++) {
            for (int subr = 0; subr < J->nrows; subr++) {
                for (int subc = 0; subc < J->ncols; subc++) {
                    Jdata[r * ncols + c] += Jdata[subr * J->ncols + subc] * Jdata[subc * J->nrows + subr];
                }
            }
            Jdata[r * ncols + c] *= W[i++];
        }
    }
    free(Jdata);
}


void implicitJWJt::matVecMult(double x[], double r[]) {
    double *tmp = (double *) malloc(sizeof(double) * J->ncols);
    J->matTransVecMult(x, tmp); // tmp = Jt * x
#ifdef DEBUG
    printf("tmp = Jt * x\n");
    for (int i = 0; i < J->ncols; i++){
      printf("\ttmp[%i] = %.2f\n", i, tmp[i]);
    }
#endif
    vecMultComp(J->ncols, tmp, W); //tmp = W * Jt * x
#ifdef DEBUG
    printf("tmp = W * Jt * x\n");
    for (int i = 0; i < J->ncols; i++){
      printf("\ttmp[%i] = %.2f\n", i, tmp[i]);
    }
#endif
    J->matVecMult(tmp, r); //r = J * W * Jt * x
#ifdef DEBUG
    printf("r = J * W * Jt * x\n");
    for (int i = 0; i < J->nrows; i++){
      printf("\tr[%i] = %.2f\n", i, r[i]);
    }
#endif
    free(tmp);
}

void vecAddEqual(int n, double r[], double v[]) {
    for (int i = 0; i < n; i++) {
        r[i] = r[i] + v[i];
    }
}

void vecDiffEqual(int n, double r[], double v[]) {
    for (int i = 0; i < n; i++) {
        r[i] = r[i] - v[i];
    }
}

void vecAssign(int n, double v1[], double v2[]) {
    std::memcpy(v1, v2, sizeof(double) * n);
}

void vecTimesScalar(int n, double v[], double s) {
    for (int i = 0; i < n; i++) {
        v[i] *= s;
    }
}

double vecDot(int n, double v1[], double v2[]) {
    double dot = 0;
    for (int i = 0; i < n; i++) {
        dot += v1[i] * v2[i];
    }
    return dot;
}

double vecSqrLen(int n, double v[]) {
    return vecDot(n, v, v);
}


void vecMultComp(int n, double x[], double y[]) {
    for (int i = 0; i < n; i++) { x[i] *= y[i]; }
}

double ConjGrad(int n, implicitMatrix *A, double x[], double b[],
                double epsilon,    // how low should we go?
                int *steps) {
    int i, iMax;
    double alpha, beta, rSqrLen, rSqrLenOld, u;

    double *r = (double *) malloc(sizeof(double) * n);
    double *d = (double *) malloc(sizeof(double) * n);
    double *t = (double *) malloc(sizeof(double) * n);
    double *temp = (double *) malloc(sizeof(double) * n);

    vecAssign(n, x, b); // x = b; -> populate x in advance for warm start, we use previous iteration lambda

    vecAssign(n, r, b); // r = b;
#ifdef DEBUG
    for (int j = 0; j < n; j++){
      printf(" mult1 x[%i]=%.2f\n", j, x[j]);
    }
#endif
    A->matVecMult(x, temp); // temp = A*x;
    vecDiffEqual(n, r, temp); // r = r - temp = r - A*x;
#ifdef DEBUG
    for (int j = 0; j < n; j++){
      printf(" r= r-JWJt*x r[%i]=%.2f\n", j, r[j]);
    }
#endif
    rSqrLen = vecSqrLen(n, r); // rSqrLen = dot(r, r)
#ifdef DEBUG
    printf(" rSqrLen=%.2f\n", rSqrLen);
#endif
    vecAssign(n, d, r); // d = r
#ifdef DEBUG
    for (int j = 0; j < n; j++){
        printf(" d=r d[%i]=%.2f\n", j, d[j]);
    }
#endif
    i = 0;
    if (*steps) {
        iMax = *steps;
    } else {
        iMax = MAX_STEPS;
    }

    if (rSqrLen > epsilon)
#ifdef DEBUG
        printf(" rSqrLen > epsilon : %.6f > %.6f == %s\n", rSqrLen, epsilon, rSqrLen > epsilon ? "true" : "false");
#endif
    {
        while (i < iMax) {
            i++;
#ifdef DEBUG
            for (int j = 0; j < n; j++){
              printf(" mult2 d[%i]=%.2f\n", j, d[j]);
            }
#endif
            A->matVecMult(d, t); //t = A*d
            u = vecDot(n, d, t); //u = dot(d, t)

            if (u == 0) {
                printf("(SolveConjGrad iter %i) d'Ad = 0\n", i);
                break;
            }

            // How far should we go?
            alpha = rSqrLen / u;
#ifdef DEBUG
            printf(" alpha = rSqrLen / u : %.5f = %.5f / %.15f\n", alpha, rSqrLen, u);
#endif
            // Take a step along direction d
            vecAssign(n, temp, d); // temp = d
            vecTimesScalar(n, temp, alpha); //temp *= alpha
            vecAddEqual(n, x, temp); // x += temp = alpha * d

            if (i & 0x3F) {
                vecAssign(n, temp, t);       // temp = t
                vecTimesScalar(n, temp, alpha); // temp *= alpha
                vecDiffEqual(n, r, temp); //r -= temp = alpha*t
            } else {
                // For stability, correct r every 64th iteration
                vecAssign(n, r, b);
#ifdef DEBUG
                for (int j = 0; j < n; j++){
                  printf(" stab x[%i]=%.2f\n", j, x[j]);
                }
#endif
                A->matVecMult(x, temp);
                vecDiffEqual(n, r, temp);
            }

            rSqrLenOld = rSqrLen;
            rSqrLen = vecSqrLen(n, r);

            // Converged! Let's get out of here
            if (rSqrLen <= epsilon) {
                break;
            }

            // Change direction: d = r + beta * d
            beta = rSqrLen / rSqrLenOld;
#ifdef DEBUG
            printf(" beta=%.5f rSqrLen=%.5f rSqrLenOld=%.5f\n", beta, rSqrLen, rSqrLenOld);
#endif
            vecTimesScalar(n, d, beta);
            vecAddEqual(n, d, r);
        }
    }

    // free memory

    free(r);
    free(d);
    free(t);
    free(temp);

    *steps = i;
    return (rSqrLen);
}



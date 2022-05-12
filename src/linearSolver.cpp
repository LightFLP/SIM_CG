#include "linearSolver.h"

// vector helper functions

void vecAddEqual(int n, double r[], double v[])
{
  for (int i = 0; i < n; i++)
    r[i] = r[i] + v[i];
}

void vecDiffEqual(int n, double r[], double v[])
{
  for (int i = 0; i < n; i++)
    r[i] = r[i] - v[i];
}

void vecAssign(int n, double v1[], double v2[])
{
  for (int i = 0; i < n; i++)
    v1[i] = v2[i];
}

void vecTimesScalar(int n, double v[], double s)
{
  for (int i = 0; i < n; i++)
    v[i] *= s;
}

double vecDot(int n, double v1[], double v2[])
{
  double dot = 0;
  for (int i = 0; i < n; i++)
    dot += v1[i] * v2[i];
  return dot;
}

double vecSqrLen(int n, double v[])
{
  return vecDot(n, v, v);
}

double ConjGrad(int n, implicitMatrix *A, double x[], double b[], 
		double epsilon,	// how low should we go?
		int    *steps)
{
  int		i, iMax;
  double	alpha, beta, rSqrLen, rSqrLenOld, u;

  double *r = (double *) malloc(sizeof(double) * n);
  double *d = (double *) malloc(sizeof(double) * n);
  double *t = (double *) malloc(sizeof(double) * n);
  double *temp = (double *) malloc(sizeof(double) * n);

  //vecAssign(n, x, b); // x = b; -> populate x in advance for warm start, we use previous iteration lambda

  vecAssign(n, r, b); // r = b;
  A->matVecMult(x, temp); // temp = A*x;
  vecDiffEqual(n, r, temp); // r = r - temp = r - A*x;

  rSqrLen = vecSqrLen(n, r); // rSqrLen = dot(r, r)

  vecAssign(n, d, r); // d = r

  i = 0;
  if (*steps)
    iMax = *steps;
  else
    iMax = MAX_STEPS;
		
  if (rSqrLen > epsilon)
    while (i < iMax) {	
      i++;
      A->matVecMult(d, t); //t = A*d
      u = vecDot(n, d, t); //u = dot(d, t)
      
      if (u == 0) {
        printf("(SolveConjGrad) d'Ad = 0\n");
        break;
      }
      
      // How far should we go?
      alpha = rSqrLen / u;
      
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
        A->matVecMult(x, temp);
        vecDiffEqual(n, r, temp);
      }
      
      rSqrLenOld = rSqrLen;
      rSqrLen = vecSqrLen(n, r);
      
      // Converged! Let's get out of here
      if (rSqrLen <= epsilon)
	      break;			    
      
      // Change direction: d = r + beta * d
      beta = rSqrLen/rSqrLenOld;
      vecTimesScalar(n, d, beta);
      vecAddEqual(n, d, r);
    }
  
  // free memory

  free(r);
  free(d);
  free(t);
  free(temp);
		
  *steps = i;
  return(rSqrLen);
}



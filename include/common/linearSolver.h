#ifndef LINEAR_SOLVER_H
#define LINEAR_SOLVER_H

#include <math.h> 
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <cstring>
#include <vector>

// Karen's CGD

#define MAX_STEPS 10000

struct MatrixBlock{
  int base_row, base_col, nrows, ncols;
  // data stored like:
  // (row, col) -> index given by (row-base_row)*ncols + (col-base_col)%ncols
  std::vector<double> data;
  MatrixBlock(int base_row, int base_col, int nrows, int ncols) : 
    base_row(base_row), base_col(base_col), nrows(nrows), ncols(ncols){
      data = std::vector<double>(nrows*ncols);
    }
};

// Matrix class the solver will accept
class implicitMatrix
{
 public:
    int ncols;
    int nrows;
    std::vector<MatrixBlock*> blocks = std::vector<MatrixBlock*>();
    // r=A*x
    virtual void matVecMult(double x[], double r[], bool verbose = false) = 0;
};

// Matrix class the solver will accept
class implicitMatrixWithTrans : public implicitMatrix
{
 public:
  int ncols;
  int nrows;
  implicitMatrixWithTrans(int nrows, int ncols) : nrows(nrows), ncols(ncols){}
  
  virtual void matVecMult(double x[], double r[], bool verbose = false){
    std::memset(r, 0.0, sizeof(double) * nrows);
    for (MatrixBlock* mb : blocks){
      int i = 0;
      if (verbose) printf("MB: base_row=%i base_col=%i nrow=%i ncols=%i \n", mb->base_row, mb->base_col, mb->nrows, mb->ncols);
      for (int row = mb->base_row; row < mb->base_row + mb->nrows; ++row){
        for (int col = mb->base_col; col < mb->base_col + mb->ncols; ++col){
          if (verbose) printf("    data[%i] = %.3f \n", i, mb->data[i]);
          r[row] += x[col] * mb->data[i++];  
        }
      }
    }
  };
  
  // r = A'*x
  virtual void matTransVecMult(double x[], double r[], bool verbose = false){
    std::memset(r, 0.0, sizeof(double) * ncols);
    for (MatrixBlock* mb : blocks){
      int i = 0;
      if (verbose) printf("MB: base_row=%i base_col=%i nrow=%i ncols=%i \n", mb->base_row, mb->base_col, mb->nrows, mb->ncols);
      for (int row = mb->base_row; row < mb->base_row + mb->nrows; ++row){
        for (int col = mb->base_col; col < mb->base_col + mb->ncols; ++col){
          if (verbose) printf("    data[%i] = %.3f \n", i, mb->data[i]);
          r[col] += x[row] * mb->data[i++];  
        }
      }
    }
  };
};


class implicitJWJt : public implicitMatrix
{
    implicitMatrixWithTrans* J;
    private:
      static void vecMultComp(int n, double x[], double y[]){
        for (int i = 0; i < n; i++) x[i] *= y[i];
      }

  public:
    double* W;
    implicitJWJt(implicitMatrixWithTrans* J) : J(J){}
    virtual void matVecMult(double x[], double r[], bool verbose = false){
        double* tmp = (double*) malloc(sizeof(double) *J->ncols);
        J->matTransVecMult(x, tmp); // tmp = Jt * x
        vecMultComp(J->ncols, tmp, W); //tmp = W * Jt * x
        J->matVecMult(tmp, r); //r = J * W * Jt * x
    }
};



// Solve Ax = b for a symmetric, positive definite matrix A
// A is represented implicitely by the function "matVecMult"
// which performs a matrix vector multiple Av and places result in r
// "n" is the length of the vectors x and b
// "epsilon" is the error tolerance
// "steps", as passed, is the maximum number of steps, or 0 (implying MAX_STEPS)
// Upon completion, "steps" contains the number of iterations taken
double ConjGrad(int n, implicitMatrix *A, double x[], double b[], 
		double epsilon,	// how low should we go?
		int    *steps);

// Some vector helper functions
void vecAddEqual(int n, double r[], double v[]);
void vecDiffEqual(int n, double r[], double v[]);
void vecAssign(int n, double v1[], double v2[]);
void vecTimesScalar(int n, double v[], double s);
double vecDot(int n, double v1[], double v2[]);
double vecSqrLen(int n, double v[]);


#endif

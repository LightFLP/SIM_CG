#pragma once
// #define DEBUG
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
  double* data;
  MatrixBlock(int base_row, int base_col, int nrows, int ncols) : 
    base_row(base_row), base_col(base_col), nrows(nrows), ncols(ncols){
      data = (double*) malloc(sizeof(double) *nrows*ncols);
      //printf(" MatrixBlock d=%i\n", data);
    }

};

// Matrix class the solver will accept
class implicitMatrix
{
 public:
    int nrows;
    int ncols;
    std::vector<MatrixBlock> blocks = std::vector<MatrixBlock>();
    // r=A*x
    virtual void matVecMult(double x[], double r[]) = 0;
    void Clear(){
      for (MatrixBlock& mb : blocks){
        //printf("~MatrixBlock d=%i\n", mb.data);
        free(mb.data);
      }
      blocks.clear();
      // printf("implicitMatrix.Clear()\n");
    }

    virtual void to_matrix(double* data){
      for (MatrixBlock& b : blocks){
        int i = 0;
        for (int r = b.base_row; r < b.base_row + b.nrows; r++){
          for (int c = b.base_col; c < b.base_col + b.ncols; c++){
            data[r * ncols + c] += b.data[i++];
          }
        }
      }
    }

    virtual void print_matrix(){
      double* data = (double*) malloc(sizeof(double) * nrows * ncols);
      std::memset(data, 0.0, nrows*ncols*sizeof(double));
      to_matrix(data);
      printf("\tnrows=%i ncols=%i\n", nrows, ncols); 
      int i = 0;
      for (int r = 0; r < nrows; r++){
        for (int c = 0; c < ncols; c++){
          printf("%.4f\t", data[i++]);
        }
        printf("\n");
      }

      free(data);
    }

    implicitMatrix(int nrows, int ncols) : nrows(nrows), ncols(ncols){};
    implicitMatrix(){};
    void AddBlocks(std::vector<MatrixBlock> bs){
      for (MatrixBlock& b : bs){
          blocks.push_back(b);
      } 
    }
};

// Matrix class the solver will accept
class implicitMatrixWithTrans : public implicitMatrix
{
 public:
  implicitMatrixWithTrans(int nrows, int ncols) : implicitMatrix(nrows, ncols){};
  
  void matVecMult(double x[], double r[]){
    std::memset(r, 0.0, sizeof(double) * nrows);
    for (MatrixBlock& mb : blocks){
      int i = 0;
      for (int row = mb.base_row; row < mb.base_row + mb.nrows; ++row){
        for (int col = mb.base_col; col < mb.base_col + mb.ncols; ++col){
          r[row] += x[col] * mb.data[i++];  
        }
      }
    }
  };
  
  // r = A'*x
  void matTransVecMult(double x[], double r[]){
    std::memset(r, 0.0, sizeof(double) * ncols);
    for (MatrixBlock& mb : blocks){
      int i = 0;
      for (int row = mb.base_row; row < mb.base_row + mb.nrows; ++row){
        for (int col = mb.base_col; col < mb.base_col + mb.ncols; ++col){
          r[col] += x[row] * mb.data[i++];  
        }
      }
    }
  };
};



class implicitJWJt : public implicitMatrix
{
    implicitMatrixWithTrans* J;

  public:
    double* W;
    implicitJWJt(implicitMatrixWithTrans* _J) : J(_J){
      nrows = _J->nrows;
      ncols = _J->nrows;
    };
    void matVecMult(double x[], double r[]);
    void to_matrix(double* data) override;
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
void vecMultComp(int n, double x[], double y[]);

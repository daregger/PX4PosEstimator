/*
 * kalman_dlqe1.c
 *
 * Code generation for function 'kalman_dlqe1'
 *
 * C source code generated on: Wed Feb 13 20:34:32 2013
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "kalman_dlqe1.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void kalman_dlqe1(const real32_T A[9], const real32_T C[3], const real32_T K[3],
                  const real32_T x_aposteriori_k[3], real32_T z, real32_T
                  x_aposteriori[3])
{
  real32_T y;
  int32_T i0;
  real32_T b_y[3];
  int32_T i1;
  real32_T f0;
  y = 0.0F;
  for (i0 = 0; i0 < 3; i0++) {
    b_y[i0] = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      b_y[i0] += C[i1] * A[i1 + 3 * i0];
    }

    y += b_y[i0] * x_aposteriori_k[i0];
  }

  y = z - y;
  for (i0 = 0; i0 < 3; i0++) {
    f0 = 0.0F;
    for (i1 = 0; i1 < 3; i1++) {
      f0 += A[i0 + 3 * i1] * x_aposteriori_k[i1];
    }

    x_aposteriori[i0] = f0 + K[i0] * y;
  }
}

/* End of code generation (kalman_dlqe1.c) */

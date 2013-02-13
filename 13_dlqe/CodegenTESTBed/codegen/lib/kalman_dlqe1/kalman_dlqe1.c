/*
 * kalman_dlqe1.c
 *
 * Code generation for function 'kalman_dlqe1'
 *
 * C source code generated on: Wed Feb 13 19:32:50 2013
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
                  real32_T x_aposteriori[3], real32_T z)
{
  int32_T k;
  real32_T b_A[3];
  int32_T i0;
  real32_T y;
  real32_T f0;
  for (k = 0; k < 3; k++) {
    b_A[k] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      b_A[k] += C[i0] * A[i0 + 3 * k];
    }
  }

  y = 0.0F;
  for (k = 0; k < 3; k++) {
    y += b_A[k] * x_aposteriori[k];
  }

  y = z - y;
  for (k = 0; k < 3; k++) {
    f0 = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      f0 += A[k + 3 * i0] * x_aposteriori[i0];
    }

    b_A[k] = f0 + K[k] * y;
  }

  for (k = 0; k < 3; k++) {
    x_aposteriori[k] = b_A[k];
  }
}

/* End of code generation (kalman_dlqe1.c) */

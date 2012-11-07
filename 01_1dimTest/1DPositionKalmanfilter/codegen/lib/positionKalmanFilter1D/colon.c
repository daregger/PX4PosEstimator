/*
 * colon.c
 *
 * Code generation for function 'colon'
 *
 * C source code generated on: Wed Nov 07 09:42:49 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "positionKalmanFilter1D.h"
#include "colon.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void eml_signed_integer_colon(int32_T b, int32_T y_data[2], int32_T y_size[2])
{
  int32_T yk;
  int32_T k;
  y_size[0] = 1;
  y_size[1] = b;
  y_data[0] = 1;
  yk = 1;
  k = 2;
  while (k <= b) {
    yk++;
    y_data[1] = yk;
    k = 3;
  }
}

/* End of code generation (colon.c) */

/*
 * positionKalmanFilter1D.h
 *
 * Code generation for function 'positionKalmanFilter1D'
 *
 * C source code generated on: Fri Nov 02 17:08:09 2012
 *
 */

#ifndef __POSITIONKALMANFILTER1D_H__
#define __POSITIONKALMANFILTER1D_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include "rt_nonfinite.h"

#include "rtwtypes.h"
#include "positionKalmanFilter1D_types.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern void positionKalmanFilter1D(const real32_T x_aposteriori_k[2], const real32_T P_aposteriori_k[4], uint8_T velocity_observe, uint8_T gps_update, real32_T acc, const real32_T z[2], const real32_T sigma[2], const real32_T q[2], real32_T dt, real32_T x_aposteriori[2], real32_T P_aposteriori[4]);
#endif
/* End of code generation (positionKalmanFilter1D.h) */

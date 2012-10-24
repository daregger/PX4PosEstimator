/*
 * positionKalmanFilter1D.c
 *
 * Code generation for function 'positionKalmanFilter1D'
 *
 * C source code generated on: Wed Oct 17 09:00:38 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "positionKalmanFilter1D.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real32_T rt_powf_snf(real32_T u0, real32_T u1);

/* Function Definitions */
static real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T f0;
  real32_T f1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f0 = (real32_T)fabs(u0);
    f1 = (real32_T)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f0 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f0 > 1.0F) {
        if (u1 > 0.0F) {
          y = ((real32_T)rtInf);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = ((real32_T)rtInf);
      }
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = (real32_T)sqrt(u0);
    } else if ((u0 < 0.0F) && (u1 > (real32_T)floor(u1))) {
      y = ((real32_T)rtNaN);
    } else {
      y = (real32_T)pow(u0, u1);
    }
  }

  return y;
}

void positionKalmanFilter1D(const real32_T x_aposteriori_k[2], const real32_T
  P_aposteriori_k[4], uint8_T velocity_observe, uint8_T gps_update, real32_T acc,
  const real32_T z[2], const real32_T sigma[2], const real32_T q[2], real32_T dt,
  real32_T x_aposteriori[2], real32_T P_aposteriori[4])
{
  real32_T A[4];
  int32_T i;
  int8_T H[4];
  static const int8_T iv0[4] = { 1, 0, 0, 0 };

  static const int8_T iv1[4] = { 1, 0, 0, 1 };

  real32_T b_z[2];
  real32_T x_apriori[2];
  real32_T r;
  int32_T i0;
  real32_T b_A[4];
  real32_T c_A[4];
  int32_T i1;
  real32_T P_apriori[4];
  real32_T t;
  int8_T I[4];

  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
  /*  Kalman Filter for Position Estimator from Acc + GPS */
  /*   */
  /*  state_vector x_aposteriori_k = [pos vel]' */
  /*  P_aposteriori_k = P */
  /*  velocitiy_observe = vo */
  /*  gps_update = u */
  /*  acc = a */
  /*  z = [pos_meas vel_meas] */
  /*  sigma = [sigma1 sigma2] */
  /*  q = [q(1) q(2)]' */
  /*  dt */
  /*  $Author: Damian Aregger $    $Date: 2012 $    $Revision: 1 $ */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\ */
  /*  process noise variance */
  /*  measurement noise variance */
  /*  system description */
  A[0] = 1.0F;
  A[2] = dt;
  for (i = 0; i < 2; i++) {
    A[1 + (i << 1)] = (real32_T)i;
  }

  /*  decide if velocity can be observed */
  if (velocity_observe == 0) {
    for (i = 0; i < 4; i++) {
      H[i] = iv0[i];
    }
  } else {
    for (i = 0; i < 4; i++) {
      H[i] = iv1[i];
    }
  }

  /*     %% Main Filter step */
  /* Step 1 prediction/a priori */
  b_z[0] = 0.0F;
  b_z[1] = acc;
  for (i = 0; i < 2; i++) {
    r = 0.0F;
    for (i0 = 0; i0 < 2; i0++) {
      r += A[i + (i0 << 1)] * x_aposteriori_k[i0];
    }

    x_apriori[i] = r + b_z[i];
  }

  /* project state ahead */
  for (i = 0; i < 2; i++) {
    for (i0 = 0; i0 < 2; i0++) {
      b_A[i + (i0 << 1)] = 0.0F;
      for (i1 = 0; i1 < 2; i1++) {
        b_A[i + (i0 << 1)] += A[i + (i1 << 1)] * P_aposteriori_k[i1 + (i0 << 1)];
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      c_A[i + (i0 << 1)] = 0.0F;
      for (i1 = 0; i1 < 2; i1++) {
        c_A[i + (i0 << 1)] += b_A[i + (i1 << 1)] * A[i0 + (i1 << 1)];
      }
    }
  }

  b_A[0] = rt_powf_snf(q[0], 2.0F) * dt;
  b_A[2] = 0.0F;
  b_A[1] = 0.0F;
  b_A[3] = rt_powf_snf(q[1], 2.0F) * dt;
  for (i = 0; i < 2; i++) {
    for (i0 = 0; i0 < 2; i0++) {
      P_apriori[i0 + (i << 1)] = c_A[i0 + (i << 1)] + b_A[i0 + (i << 1)];
    }
  }

  /* project error covariance ahead */
  /* Step 2 correction/a posteriori */
  if (gps_update == 1) {
    for (i = 0; i < 2; i++) {
      for (i0 = 0; i0 < 2; i0++) {
        b_A[i + (i0 << 1)] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          b_A[i + (i0 << 1)] += (real32_T)H[i + (i1 << 1)] * P_apriori[i1 + (i0 <<
            1)];
        }
      }

      for (i0 = 0; i0 < 2; i0++) {
        c_A[i + (i0 << 1)] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          c_A[i + (i0 << 1)] += b_A[i + (i1 << 1)] * (real32_T)H[i1 + (i0 << 1)];
        }
      }
    }

    b_A[0] = rt_powf_snf(sigma[0], 2.0F);
    b_A[2] = 0.0F;
    b_A[1] = 0.0F;
    b_A[3] = rt_powf_snf(sigma[1], 2.0F);
    for (i = 0; i < 2; i++) {
      for (i0 = 0; i0 < 2; i0++) {
        A[i0 + (i << 1)] = c_A[i0 + (i << 1)] + b_A[i0 + (i << 1)];
      }
    }

    if ((real32_T)fabs(A[1]) > (real32_T)fabs(A[0])) {
      r = A[0] / A[1];
      t = 1.0F / (r * A[3] - A[2]);
      b_A[0] = A[3] / A[1] * t;
      b_A[1] = -t;
      b_A[2] = -A[2] / A[1] * t;
      b_A[3] = r * t;
    } else {
      r = A[1] / A[0];
      t = 1.0F / (A[3] - r * A[2]);
      b_A[0] = A[3] / A[0] * t;
      b_A[1] = -r * t;
      b_A[2] = -A[2] / A[0] * t;
      b_A[3] = t;
    }

    for (i = 0; i < 2; i++) {
      for (i0 = 0; i0 < 2; i0++) {
        c_A[i + (i0 << 1)] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          c_A[i + (i0 << 1)] += P_apriori[i + (i1 << 1)] * (real32_T)H[i0 + (i1 <<
            1)];
        }
      }

      for (i0 = 0; i0 < 2; i0++) {
        A[i + (i0 << 1)] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          A[i + (i0 << 1)] += c_A[i + (i1 << 1)] * b_A[i1 + (i0 << 1)];
        }
      }
    }

    /* compute kalman gain */
    for (i = 0; i < 2; i++) {
      r = 0.0F;
      for (i0 = 0; i0 < 2; i0++) {
        r += (real32_T)H[i + (i0 << 1)] * x_apriori[i0];
      }

      b_z[i] = z[i] - r;
    }

    for (i = 0; i < 2; i++) {
      r = 0.0F;
      for (i0 = 0; i0 < 2; i0++) {
        r += A[i + (i0 << 1)] * b_z[i0];
      }

      x_aposteriori[i] = x_apriori[i] + r;
    }

    /* update estimate via z */
    for (i = 0; i < 4; i++) {
      I[i] = 0;
    }

    for (i = 0; i < 2; i++) {
      I[i + (i << 1)] = 1;
    }

    for (i = 0; i < 2; i++) {
      for (i0 = 0; i0 < 2; i0++) {
        r = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          r += A[i + (i1 << 1)] * (real32_T)H[i1 + (i0 << 1)];
        }

        b_A[i + (i0 << 1)] = (real32_T)I[i + (i0 << 1)] - r;
      }
    }

    for (i = 0; i < 2; i++) {
      for (i0 = 0; i0 < 2; i0++) {
        P_aposteriori[i + (i0 << 1)] = 0.0F;
        for (i1 = 0; i1 < 2; i1++) {
          P_aposteriori[i + (i0 << 1)] += b_A[i + (i1 << 1)] * P_apriori[i1 +
            (i0 << 1)];
        }
      }
    }

    /* update error covariance */
  } else {
    for (i = 0; i < 2; i++) {
      x_aposteriori[i] = x_apriori[i];
    }

    for (i = 0; i < 4; i++) {
      P_aposteriori[i] = P_apriori[i];
    }
  }
}

/* End of code generation (positionKalmanFilter1D.c) */

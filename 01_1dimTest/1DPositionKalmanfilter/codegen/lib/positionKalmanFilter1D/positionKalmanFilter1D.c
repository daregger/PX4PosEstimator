/*
 * positionKalmanFilter1D.c
 *
 * Code generation for function 'positionKalmanFilter1D'
 *
 * C source code generated on: Fri Nov 02 17:08:09 2012
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
static real32_T eml_div(real32_T x, real32_T y);
static void eml_lusolve(const real32_T A_data[4], const int32_T A_size[2],
  real32_T B_data[4], int32_T B_size[2]);
static void eml_matlab_zlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau,
  real32_T C_data[4], int32_T C_size[2], int32_T ic0, int32_T ldc, real32_T
  work_data[2], int32_T work_size[1]);
static void eml_qrsolve(const real32_T A_data[4], const int32_T A_size[2],
  real32_T B_data[4], int32_T B_size[2], real32_T Y_data[4], int32_T Y_size[2]);
static real32_T eml_xnrm2(int32_T n, const real32_T x_data[4], const int32_T
  x_size[2], int32_T ix0);
static real32_T rt_hypotf_snf(real32_T u0, real32_T u1);
static real32_T rt_powf_snf(real32_T u0, real32_T u1);

/* Function Definitions */
static real32_T eml_div(real32_T x, real32_T y)
{
  return x / y;
}

static void eml_lusolve(const real32_T A_data[4], const int32_T A_size[2],
  real32_T B_data[4], int32_T B_size[2])
{
  int32_T iy;
  int32_T c;
  real32_T b_A_data[4];
  int32_T u1;
  int32_T ipiv_size[2];
  int32_T ipiv_data[2];
  int32_T ix;
  int32_T jy;
  real32_T temp;
  int32_T jA;
  iy = A_size[0] * A_size[1] - 1;
  for (c = 0; c <= iy; c++) {
    b_A_data[c] = A_data[c];
  }

  iy = A_size[1];
  u1 = A_size[1];
  if (iy <= u1) {
    u1 = iy;
  }

  eml_signed_integer_colon(u1, ipiv_data, ipiv_size);
  iy = A_size[1] - 1;
  u1 = A_size[1];
  if (iy <= u1) {
    u1 = iy;
  }

  ix = 1;
  while (ix <= u1) {
    iy = 0;
    if ((A_size[1] > 1) && ((real32_T)fabs(b_A_data[1]) > (real32_T)fabs
                            (b_A_data[0]))) {
      iy = 1;
    }

    if (b_A_data[iy] != 0.0F) {
      if (iy != 0) {
        ipiv_data[0] = 2;
        ix = 0;
        iy = 1;
        for (jy = 1; jy <= A_size[1]; jy++) {
          temp = b_A_data[ix];
          b_A_data[ix] = b_A_data[iy];
          b_A_data[iy] = temp;
          ix += A_size[1];
          iy += A_size[1];
        }
      }

      jA = 2;
      while (jA <= A_size[1]) {
        b_A_data[1] /= b_A_data[0];
        jA = 3;
      }
    }

    jA = A_size[1] + 1;
    jy = A_size[1];
    ix = 1;
    while (ix <= A_size[1] - 1) {
      temp = b_A_data[jy];
      if (b_A_data[jy] != 0.0F) {
        ix = 1;
        c = A_size[1] + jA;
        for (iy = jA; iy + 1 <= c - 1; iy++) {
          b_A_data[iy] += b_A_data[ix] * -temp;
          ix++;
        }
      }

      jy += A_size[1];
      jA += A_size[1];
      ix = 2;
    }

    ix = 2;
  }

  for (jA = 0; jA + 1 <= A_size[1]; jA++) {
    if (ipiv_data[jA] != jA + 1) {
      for (ix = 0; ix < 2; ix++) {
        temp = B_data[jA + B_size[0] * ix];
        B_data[jA + B_size[0] * ix] = B_data[(ipiv_data[jA] + B_size[0] * ix) -
          1];
        B_data[(ipiv_data[jA] + B_size[0] * ix) - 1] = temp;
      }
    }
  }

  for (ix = 0; ix < 2; ix++) {
    iy = A_size[1] * ix;
    for (jy = 0; jy + 1 <= A_size[1]; jy++) {
      c = A_size[1] * jy;
      if (B_data[jy + iy] != 0.0F) {
        jA = jy + 2;
        while (jA <= A_size[1]) {
          B_data[1 + iy] -= B_data[jy + iy] * b_A_data[c + 1];
          jA = 3;
        }
      }
    }
  }

  for (ix = 0; ix < 2; ix++) {
    iy = A_size[1] * ix;
    for (jy = A_size[1] - 1; jy + 1 > 0; jy--) {
      c = A_size[1] * jy;
      if (B_data[jy + iy] != 0.0F) {
        B_data[jy + iy] /= b_A_data[jy + c];
        jA = 1;
        while (jA <= jy) {
          B_data[iy] -= B_data[jy + iy] * b_A_data[c];
          jA = 2;
        }
      }
    }
  }
}

static void eml_matlab_zlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau,
  real32_T C_data[4], int32_T C_size[2], int32_T ic0, int32_T ldc, real32_T
  work_data[2], int32_T work_size[1])
{
  int32_T lastv;
  int32_T i;
  int32_T lastc;
  boolean_T exitg2;
  int32_T exitg1;
  int32_T iy;
  int32_T jA;
  int32_T i1;
  int32_T ix;
  real32_T c;
  int32_T jy;
  if (tau != 0.0F) {
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C_data[i - 2] == 0.0F)) {
      lastv--;
      i--;
    }

    lastc = n;
    exitg2 = FALSE;
    while ((exitg2 == 0U) && (lastc > 0)) {
      i = ic0;
      do {
        exitg1 = 0;
        if (i <= (ic0 + lastv) - 1) {
          if (C_data[i - 1] != 0.0F) {
            exitg1 = 1;
          } else {
            i++;
          }
        } else {
          lastc = 0;
          exitg1 = 2;
        }
      } while (exitg1 == 0U);

      if (exitg1 == 1U) {
        exitg2 = TRUE;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    if (lastc == 0) {
    } else {
      i = lastc - 1;
      iy = 0;
      jA = 1;
      while (jA <= i + 1) {
        work_data[0] = 0.0F;
        jA = 2;
      }

      i1 = ic0 + ldc * i;
      for (jA = ic0; jA <= i1; jA += ldc) {
        ix = iv0;
        c = 0.0F;
        jy = jA + lastv;
        for (i = jA; i <= jy - 1; i++) {
          c += C_data[i - 1] * C_data[ix - 1];
          ix++;
        }

        work_data[iy] += c;
        iy++;
      }
    }

    if (-tau == 0.0F) {
    } else {
      jA = ic0 - 1;
      jy = 0;
      i = 1;
      while (i <= lastc) {
        if (work_data[jy] != 0.0F) {
          c = work_data[jy] * -tau;
          ix = iv0;
          i1 = lastv + jA;
          for (i = jA; i + 1 <= i1; i++) {
            C_data[i] += C_data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        jA += ldc;
        i = 2;
      }
    }
  }
}

static void eml_qrsolve(const real32_T A_data[4], const int32_T A_size[2],
  real32_T B_data[4], int32_T B_size[2], real32_T Y_data[4], int32_T Y_size[2])
{
  real_T u0;
  real_T u1;
  int32_T mn;
  int32_T b_A_size[2];
  int32_T ix;
  int32_T pvt;
  real32_T b_A_data[4];
  int32_T b_mn;
  real32_T tau_data[2];
  int32_T jpvt_size[2];
  int32_T jpvt_data[2];
  int32_T work_size[1];
  real32_T work_data[2];
  real32_T vn1_data[2];
  real32_T vn2_data[2];
  int32_T k;
  int32_T i;
  int32_T i_i;
  int32_T nmi;
  int32_T mmi;
  int32_T iy;
  real32_T wj;
  real32_T atmp;
  real32_T f1;
  real_T rankR;
  boolean_T exitg1;
  u0 = (real_T)A_size[0];
  u1 = (real_T)A_size[1];
  if (u0 <= u1) {
    u1 = u0;
  }

  mn = (int32_T)u1 - 1;
  b_A_size[0] = A_size[0];
  b_A_size[1] = A_size[1];
  ix = A_size[0] * A_size[1] - 1;
  for (pvt = 0; pvt <= ix; pvt++) {
    b_A_data[pvt] = A_data[pvt];
  }

  ix = A_size[0];
  b_mn = A_size[1];
  if (ix <= b_mn) {
    b_mn = ix;
  }

  eml_signed_integer_colon(A_size[1], jpvt_data, jpvt_size);
  work_size[0] = (int8_T)A_size[1];
  ix = (int8_T)A_size[1] - 1;
  for (pvt = 0; pvt <= ix; pvt++) {
    work_data[pvt] = 0.0F;
  }

  k = 1;
  for (ix = 0; ix + 1 <= A_size[1]; ix++) {
    vn1_data[ix] = eml_xnrm2(A_size[0], A_data, A_size, k);
    vn2_data[ix] = vn1_data[ix];
    k += A_size[0];
  }

  for (i = 0; i + 1 <= b_mn; i++) {
    i_i = i + i * A_size[0];
    nmi = A_size[1] - i;
    mmi = (A_size[0] - i) - 1;
    if (nmi < 1) {
      ix = -1;
    } else {
      ix = 0;
      if ((nmi > 1) && ((real32_T)fabs(vn1_data[1]) > (real32_T)fabs(vn1_data[i])))
      {
        ix = 1;
      }
    }

    pvt = i + ix;
    if (pvt + 1 != i + 1) {
      ix = A_size[0] * pvt;
      iy = A_size[0] * i;
      for (k = 1; k <= A_size[0]; k++) {
        wj = b_A_data[ix];
        b_A_data[ix] = b_A_data[iy];
        b_A_data[iy] = wj;
        ix++;
        iy++;
      }

      ix = jpvt_data[pvt];
      jpvt_data[pvt] = jpvt_data[i];
      jpvt_data[i] = ix;
      vn1_data[pvt] = vn1_data[i];
      vn2_data[pvt] = vn2_data[i];
    }

    if (i + 1 < A_size[0]) {
      ix = i_i + 2;
      atmp = b_A_data[i_i];
      f1 = 0.0F;
      if (mmi + 1 <= 0) {
      } else {
        wj = eml_xnrm2(mmi, b_A_data, b_A_size, ix);
        if (wj != 0.0F) {
          wj = rt_hypotf_snf((real32_T)fabs(b_A_data[i_i]), wj);
          if (b_A_data[i_i] >= 0.0F) {
            wj = -wj;
          }

          if ((real32_T)fabs(wj) < 9.86076132E-32F) {
            iy = 0;
            do {
              iy++;
              pvt = (ix + mmi) - 1;
              for (k = ix; k <= pvt; k++) {
                b_A_data[k - 1] *= 1.01412048E+31F;
              }

              wj *= 1.01412048E+31F;
              atmp *= 1.01412048E+31F;
            } while (!((real32_T)fabs(wj) >= 9.86076132E-32F));

            wj = eml_xnrm2(mmi, b_A_data, b_A_size, ix);
            wj = rt_hypotf_snf((real32_T)fabs(atmp), wj);
            if (atmp >= 0.0F) {
              wj = -wj;
            }

            f1 = (wj - atmp) / wj;
            atmp = 1.0F / (atmp - wj);
            pvt = (ix + mmi) - 1;
            while (ix <= pvt) {
              b_A_data[ix - 1] *= atmp;
              ix++;
            }

            for (k = 1; k <= iy; k++) {
              wj *= 9.86076132E-32F;
            }

            atmp = wj;
          } else {
            f1 = (wj - b_A_data[i_i]) / wj;
            atmp = 1.0F / (b_A_data[i_i] - wj);
            pvt = (ix + mmi) - 1;
            while (ix <= pvt) {
              b_A_data[ix - 1] *= atmp;
              ix++;
            }

            atmp = wj;
          }
        }
      }

      tau_data[0] = f1;
    } else {
      atmp = b_A_data[i_i];
      tau_data[i] = 0.0F;
    }

    b_A_data[i_i] = atmp;
    if (i + 1 < A_size[1]) {
      atmp = b_A_data[i_i];
      b_A_data[i_i] = 1.0F;
      eml_matlab_zlarf(mmi + 1, nmi - 1, i_i + 1, tau_data[0], b_A_data,
                       b_A_size, A_size[0] + 1, A_size[0], work_data, work_size);
      b_A_data[i_i] = atmp;
    }

    ix = i + 2;
    while (ix <= A_size[1]) {
      if (vn1_data[1] != 0.0F) {
        atmp = (real32_T)fabs(b_A_data[i + b_A_size[0]]) / vn1_data[1];
        wj = atmp * atmp;
        atmp = 1.0F - atmp * atmp;
        if (1.0F - wj < 0.0F) {
          atmp = 0.0F;
        }

        wj = vn1_data[1] / vn2_data[1];
        if (atmp * (wj * wj) <= 0.000345266977F) {
          if (i + 1 < A_size[0]) {
            wj = 0.0F;
            if (mmi < 1) {
            } else {
              wj = (real32_T)fabs(b_A_data[3]);
            }

            vn1_data[1] = wj;
            vn2_data[1] = vn1_data[1];
          } else {
            vn1_data[1] = 0.0F;
            vn2_data[1] = 0.0F;
          }
        } else {
          vn1_data[1] *= (real32_T)sqrt(atmp);
        }
      }

      ix = 3;
    }
  }

  rankR = 0.0;
  k = 0;
  exitg1 = FALSE;
  while ((exitg1 == 0U) && (k <= mn)) {
    u0 = (real_T)A_size[0];
    u1 = (real_T)A_size[1];
    if (u0 >= u1) {
      u1 = u0;
    }

    if ((real32_T)fabs(b_A_data[k + b_A_size[0] * k]) <= (real32_T)u1 *
        (real32_T)fabs(b_A_data[0]) * 1.1920929E-7F) {
      exitg1 = TRUE;
    } else {
      rankR++;
      k++;
    }
  }

  Y_size[0] = (int8_T)A_size[1];
  Y_size[1] = 2;
  ix = ((int8_T)A_size[1] << 1) - 1;
  for (pvt = 0; pvt <= ix; pvt++) {
    Y_data[pvt] = 0.0F;
  }

  for (ix = 0; ix <= mn; ix++) {
    if (tau_data[ix] != 0.0F) {
      for (k = 0; k < 2; k++) {
        wj = B_data[ix + B_size[0] * k];
        pvt = A_size[0] - ix;
        i = 0;
        while (i <= pvt - 2) {
          wj += b_A_data[1 + b_A_size[0] * ix] * B_data[1 + B_size[0] * k];
          i = 1;
        }

        wj *= tau_data[ix];
        if (wj != 0.0F) {
          B_data[ix + B_size[0] * k] -= wj;
          pvt = A_size[0] - ix;
          i = 0;
          while (i <= pvt - 2) {
            B_data[1 + B_size[0] * k] -= b_A_data[1 + b_A_size[0] * ix] * wj;
            i = 1;
          }
        }
      }
    }
  }

  for (k = 0; k < 2; k++) {
    for (i = 0; i <= (int32_T)rankR - 1; i++) {
      Y_data[(jpvt_data[(int32_T)(1.0 + (real_T)i) - 1] + Y_size[0] * k) - 1] =
        B_data[((int32_T)(1.0 + (real_T)i) + B_size[0] * k) - 1];
    }

    for (ix = 0; ix <= (int32_T)-(1.0 + (-1.0 - rankR)) - 1; ix++) {
      u0 = rankR + -(real_T)ix;
      Y_data[(jpvt_data[(int32_T)u0 - 1] + Y_size[0] * k) - 1] = eml_div(Y_data
        [(jpvt_data[(int32_T)u0 - 1] + Y_size[0] * k) - 1], b_A_data[((int32_T)
        u0 + b_A_size[0] * ((int32_T)u0 - 1)) - 1]);
      i = 0;
      while (i <= (int32_T)u0 - 2) {
        Y_data[(jpvt_data[0] + Y_size[0] * k) - 1] -= Y_data[(jpvt_data[(int32_T)
          u0 - 1] + Y_size[0] * k) - 1] * b_A_data[b_A_size[0] * ((int32_T)u0 -
          1)];
        i = 1;
      }
    }
  }
}

static real32_T eml_xnrm2(int32_T n, const real32_T x_data[4], const int32_T
  x_size[2], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T k;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  if (n < 1) {
  } else if (n == 1) {
    y = (real32_T)fabs(x_data[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    for (k = ix0; k <= ix0 + 1; k++) {
      absxk = (real32_T)fabs(x_data[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = 1.0F + y * t * t;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * (real32_T)sqrt(y);
  }

  return y;
}

static real32_T rt_hypotf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T a;
  a = (real32_T)fabs(u0);
  y = (real32_T)fabs(u1);
  if (a < y) {
    a /= y;
    y *= (real32_T)sqrt(a * a + 1.0F);
  } else if (a > y) {
    y /= a;
    y = a * (real32_T)sqrt(y * y + 1.0F);
  } else if (rtIsNaNF(y)) {
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

static real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T f2;
  real32_T f3;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else {
    f2 = (real32_T)fabs(u0);
    f3 = (real32_T)fabs(u1);
    if (rtIsInfF(u1)) {
      if (f2 == 1.0F) {
        y = ((real32_T)rtNaN);
      } else if (f2 > 1.0F) {
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
    } else if (f3 == 0.0F) {
      y = 1.0F;
    } else if (f3 == 1.0F) {
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
  real32_T R_full[4];
  real32_T A[4];
  int32_T ic;
  int32_T H_size_idx_0;
  int8_T H_data[4];
  int32_T ar;
  real32_T R_data[4];
  static const int8_T H_full[4] = { 1, 0, 0, 1 };

  real32_T b[2];
  real32_T f0;
  int32_T ia;
  real32_T b_A[4];
  real32_T c_A[4];
  int32_T i0;
  int32_T loop_ub;
  int32_T A_size[2];
  int32_T b_loop_ub;
  real32_T A_data[4];
  int32_T B_size[2];
  real32_T B_data[4];
  int32_T b_B_size[2];
  real32_T K_data[4];
  real32_T b_data[2];
  real32_T y[2];
  int8_T H[4];

  /*  function [x_aposteriori,P_aposteriori] = positionKalmanFilter1D(x_aposteriori_k,P_aposteriori_k,velocity_observe,gps_update,acc,z,sigma,q,dt) */
  /*  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
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
  /*   */
  /*      % process noise variance */
  /*      Q = [q(1)^2*dt 0; 0 q(2)^2*dt]; */
  /*      % measurement noise variance */
  /*      R = [sigma(1)^2 0; 0 sigma(2)^2]; */
  /*      % system description */
  /*      A = [1 dt; 0 1]; */
  /*      B = [0 acc]'; */
  /*      % decide if velocity can be observed */
  /*      if velocity_observe == 0 */
  /*          H = [1 0; 0 0]; */
  /*      else */
  /*          H = [1 0; 0 1]; */
  /*      end */
  /*      %% Main Filter step */
  /*      %Step 1 prediction/a priori */
  /*      x_apriori = A*x_aposteriori_k+B;                 %project state ahead */
  /*      P_apriori = A*P_aposteriori_k*A'+Q;                    %project error covariance ahead */
  /*      %Step 2 correction/a posteriori */
  /*      if gps_update == 1 */
  /*          K = P_apriori*H'*inv(H*P_apriori*H+R);              %compute kalman gain */
  /*          x_aposteriori = x_apriori+K*(z-H*x_apriori);        %update estimate via z */
  /*          P_aposteriori = (eye(2)-K*H)*P_apriori;           %update error covariance */
  /*      else */
  /*         x_aposteriori = x_apriori; */
  /*         P_aposteriori = P_apriori; */
  /*      end   */
  /*  end */
  /*  % Kalman Filter for Position Estimator from Acc + GPS */
  /*  %  */
  /*  % state_vector x_aposteriori_k = [pos vel]' */
  /*  % P_aposteriori_k = [P11 P12; P21 P22] */
  /*  % velocitiy_observe = vo */
  /*  % gps_update = u */
  /*  % acc = a */
  /*  % z = [pos_meas vel_meas]' */
  /*  % sigma = [sigma1 sigma2]' */
  /*  % q = [q(1) q(2)]' */
  /*  % dt */
  /*  % $Author: Damian Aregger $    $Date: 2012 $    $Revision: 1 $ */
  /*  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\ */
  /*  process noise variance */
  /*  measurement noise variance */
  /* R_full = [sigma(1)^2 0; 0 sigma(2)^2]; */
  R_full[0] = rt_powf_snf(sigma[0], 2.0F);
  R_full[2] = 0.0F;
  R_full[1] = 0.0F;
  R_full[3] = rt_powf_snf(sigma[1], 2.0F);

  /*  system description */
  A[0] = 1.0F;
  A[2] = dt;
  for (ic = 0; ic < 2; ic++) {
    A[1 + (ic << 1)] = (real32_T)ic;
  }

  /*  decide if velocity can be observed */
  if (velocity_observe == 0) {
    H_size_idx_0 = 1;
    for (ic = 0; ic < 2; ic++) {
      H_data[ic] = (int8_T)(1 - ic);
    }

    ar = 1;
    R_data[0] = R_full[0];
  } else {
    H_size_idx_0 = 2;
    ar = 2;
    for (ic = 0; ic < 4; ic++) {
      H_data[ic] = H_full[ic];
      R_data[ic] = R_full[ic];
    }
  }

  /*     %% Main Filter step */
  /* Step 1 prediction/a priori */
  b[0] = 0.5F * dt * dt;
  b[1] = dt;
  for (ic = 0; ic < 2; ic++) {
    f0 = 0.0F;
    for (ia = 0; ia < 2; ia++) {
      f0 += A[ic + (ia << 1)] * x_aposteriori_k[ia];
    }

    x_aposteriori[ic] = f0 + b[ic] * acc;
  }

  /* project state ahead */
  for (ic = 0; ic < 2; ic++) {
    for (ia = 0; ia < 2; ia++) {
      b_A[ic + (ia << 1)] = 0.0F;
      for (i0 = 0; i0 < 2; i0++) {
        b_A[ic + (ia << 1)] += A[ic + (i0 << 1)] * P_aposteriori_k[i0 + (ia << 1)];
      }
    }

    for (ia = 0; ia < 2; ia++) {
      c_A[ic + (ia << 1)] = 0.0F;
      for (i0 = 0; i0 < 2; i0++) {
        c_A[ic + (ia << 1)] += b_A[ic + (i0 << 1)] * A[ia + (i0 << 1)];
      }
    }
  }

  b_A[0] = rt_powf_snf(q[0], 2.0F) * dt;
  b_A[2] = 0.0F;
  b_A[1] = 0.0F;
  b_A[3] = rt_powf_snf(q[1], 2.0F) * dt;
  for (ic = 0; ic < 2; ic++) {
    for (ia = 0; ia < 2; ia++) {
      R_full[ia + (ic << 1)] = c_A[ia + (ic << 1)] + b_A[ia + (ic << 1)];
    }
  }

  /* project error covariance ahead */
  /* Step 2 correction/a posteriori */
  if (gps_update == 1) {
    /* K = P_apriori*H'*inv(H*P_apriori*H'+R);              %compute kalman gain */
    loop_ub = H_size_idx_0 - 1;
    for (ic = 0; ic <= loop_ub; ic++) {
      for (ia = 0; ia < 2; ia++) {
        b_A[ic + H_size_idx_0 * ia] = 0.0F;
        for (i0 = 0; i0 < 2; i0++) {
          b_A[ic + H_size_idx_0 * ia] += (real32_T)H_data[ic + H_size_idx_0 * i0]
            * R_full[i0 + (ia << 1)];
        }
      }
    }

    A_size[0] = H_size_idx_0;
    A_size[1] = H_size_idx_0;
    loop_ub = H_size_idx_0 - 1;
    for (ic = 0; ic <= loop_ub; ic++) {
      b_loop_ub = H_size_idx_0 - 1;
      for (ia = 0; ia <= b_loop_ub; ia++) {
        f0 = 0.0F;
        for (i0 = 0; i0 < 2; i0++) {
          f0 += b_A[ia + H_size_idx_0 * i0] * (real32_T)H_data[ic + H_size_idx_0
            * i0];
        }

        A_data[ic + A_size[0] * ia] = f0 + R_data[ia + ar * ic];
      }
    }

    B_size[0] = H_size_idx_0;
    B_size[1] = 2;
    loop_ub = H_size_idx_0 - 1;
    for (ic = 0; ic <= loop_ub; ic++) {
      for (ia = 0; ia < 2; ia++) {
        B_data[ic + B_size[0] * ia] = 0.0F;
        for (i0 = 0; i0 < 2; i0++) {
          B_data[ic + B_size[0] * ia] += R_full[ia + (i0 << 1)] * (real32_T)
            H_data[ic + H_size_idx_0 * i0];
        }
      }
    }

    if (A_size[0] == A_size[1]) {
      eml_lusolve(A_data, A_size, B_data, B_size);
    } else {
      b_B_size[0] = B_size[0];
      b_B_size[1] = 2;
      loop_ub = B_size[0] * 2 - 1;
      for (ic = 0; ic <= loop_ub; ic++) {
        b_A[ic] = B_data[ic];
      }

      eml_qrsolve(A_data, A_size, b_A, b_B_size, B_data, B_size);
    }

    loop_ub = B_size[0] - 1;
    for (ic = 0; ic <= loop_ub; ic++) {
      for (ia = 0; ia < 2; ia++) {
        K_data[ia + (ic << 1)] = B_data[ic + B_size[0] * ia];
      }
    }

    if (velocity_observe == 0) {
      loop_ub = H_size_idx_0 - 1;
      for (ic = 0; ic <= loop_ub; ic++) {
        f0 = 0.0F;
        for (ia = 0; ia < 2; ia++) {
          f0 += (real32_T)H_data[ic + H_size_idx_0 * ia] * x_aposteriori[ia];
        }

        b_data[ic] = z[0] - f0;
      }

      if ((B_size[0] == 1) || (H_size_idx_0 == 1)) {
        for (ic = 0; ic < 2; ic++) {
          y[ic] = 0.0F;
          loop_ub = H_size_idx_0 - 1;
          for (ia = 0; ia <= loop_ub; ia++) {
            y[ic] += K_data[ic + (ia << 1)] * b_data[ia];
          }
        }
      } else {
        for (ic = 0; ic < 2; ic++) {
          y[ic] = 0.0F;
        }

        ar = -1;
        for (loop_ub = 0; loop_ub < 2; loop_ub++) {
          if (b_data[loop_ub] != 0.0F) {
            ia = ar;
            for (ic = 0; ic < 2; ic++) {
              ia++;
              y[ic] += b_data[loop_ub] * K_data[ia];
            }
          }

          ar += 2;
        }
      }

      for (ic = 0; ic < 2; ic++) {
        x_aposteriori[ic] += y[ic];
      }

      /* update estimate via z */
      for (ic = 0; ic < 4; ic++) {
        H[ic] = 0;
      }

      for (ic = 0; ic < 2; ic++) {
        H[ic + (ic << 1)] = 1;
      }

      for (ic = 0; ic < 2; ic++) {
        for (ia = 0; ia < 2; ia++) {
          f0 = 0.0F;
          loop_ub = B_size[0] - 1;
          for (i0 = 0; i0 <= loop_ub; i0++) {
            f0 += K_data[ic + (i0 << 1)] * (real32_T)H_data[i0 + H_size_idx_0 *
              ia];
          }

          b_A[ic + (ia << 1)] = (real32_T)H[ic + (ia << 1)] - f0;
        }
      }

      for (ic = 0; ic < 2; ic++) {
        for (ia = 0; ia < 2; ia++) {
          P_aposteriori[ic + (ia << 1)] = 0.0F;
          for (i0 = 0; i0 < 2; i0++) {
            P_aposteriori[ic + (ia << 1)] += b_A[ic + (i0 << 1)] * R_full[i0 +
              (ia << 1)];
          }
        }
      }

      /* update error covariance */
    } else {
      for (ic = 0; ic < 2; ic++) {
        for (ia = 0; ia < 2; ia++) {
          H[ia + (ic << 1)] = H_data[ia + H_size_idx_0 * ic];
        }
      }

      for (ic = 0; ic < 2; ic++) {
        f0 = 0.0F;
        for (ia = 0; ia < 2; ia++) {
          f0 += (real32_T)H[ic + (ia << 1)] * x_aposteriori[ia];
        }

        b[ic] = z[ic] - f0;
      }

      if (B_size[0] == 1) {
        for (ic = 0; ic < 2; ic++) {
          y[ic] = 0.0F;
          for (ia = 0; ia < 2; ia++) {
            y[ic] += K_data[ic + (ia << 1)] * b[ia];
          }
        }
      } else {
        for (ic = 0; ic < 2; ic++) {
          y[ic] = 0.0F;
        }

        ar = -1;
        for (loop_ub = 0; loop_ub < 2; loop_ub++) {
          if (b[loop_ub] != 0.0F) {
            ia = ar;
            for (ic = 0; ic < 2; ic++) {
              ia++;
              y[ic] += b[loop_ub] * K_data[ia];
            }
          }

          ar += 2;
        }
      }

      for (ic = 0; ic < 2; ic++) {
        x_aposteriori[ic] += y[ic];
      }

      /* update estimate via z */
      for (ic = 0; ic < 4; ic++) {
        H[ic] = 0;
      }

      for (ic = 0; ic < 2; ic++) {
        H[ic + (ic << 1)] = 1;
      }

      for (ic = 0; ic < 2; ic++) {
        for (ia = 0; ia < 2; ia++) {
          f0 = 0.0F;
          loop_ub = B_size[0] - 1;
          for (i0 = 0; i0 <= loop_ub; i0++) {
            f0 += K_data[ic + (i0 << 1)] * (real32_T)H_data[i0 + H_size_idx_0 *
              ia];
          }

          b_A[ic + (ia << 1)] = (real32_T)H[ic + (ia << 1)] - f0;
        }
      }

      for (ic = 0; ic < 2; ic++) {
        for (ia = 0; ia < 2; ia++) {
          P_aposteriori[ic + (ia << 1)] = 0.0F;
          for (i0 = 0; i0 < 2; i0++) {
            P_aposteriori[ic + (ia << 1)] += b_A[ic + (i0 << 1)] * R_full[i0 +
              (ia << 1)];
          }
        }
      }

      /* update error covariance */
    }
  } else {
    for (ic = 0; ic < 4; ic++) {
      P_aposteriori[ic] = R_full[ic];
    }
  }
}

/* End of code generation (positionKalmanFilter1D.c) */

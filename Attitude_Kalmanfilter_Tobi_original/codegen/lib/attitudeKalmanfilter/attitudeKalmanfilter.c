/*
 * attitudeKalmanfilter.c
 *
 * Code generation for function 'attitudeKalmanfilter'
 *
 * C source code generated on: Fri Oct 12 14:27:39 2012
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "attitudeKalmanfilter.h"
#include "rdivide.h"
#include "norm.h"
#include "cross.h"
#include "eye.h"
#include "mrdivide.h"
#include "diag.h"
#include "power.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
static real32_T rt_atan2f_snf(real32_T u0, real32_T u1);

/* Function Definitions */
static real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  int32_T b_u0;
  int32_T b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = (real32_T)atan2((real32_T)b_u0, (real32_T)b_u1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/*
 * function [eulerAngles,Rot_matrix,x_aposteriori,P_aposteriori] = attitudeKalmanfilter_wo(updateVect,dt,z,x_aposteriori_k,P_aposteriori_k,q,r)
 */
void attitudeKalmanfilter(const uint8_T updateVect[3], real32_T dt, const
  real32_T z[9], const real32_T x_aposteriori_k[12], const real32_T
  P_aposteriori_k[144], const real32_T q[12], const real32_T r[9], real32_T
  eulerAngles[3], real32_T Rot_matrix[9], real32_T x_aposteriori[12], real32_T
  P_aposteriori[144])
{
  real32_T a[12];
  real32_T b_a[12];
  int32_T i;
  real32_T Q[144];
  real32_T O[9];
  real_T dv0[9];
  real32_T c_a[9];
  real32_T d_a[9];
  real32_T x_n_b[3];
  real32_T b_x_aposteriori_k[3];
  real32_T m_n_b[3];
  real32_T z_n_b[3];
  real32_T x_apriori[12];
  int32_T i0;
  real_T dv1[144];
  real32_T A_lin[144];
  real32_T b_A_lin[144];
  int32_T i1;
  real32_T P_apriori[144];
  real32_T f0;
  real32_T R[81];
  real32_T b_P_apriori[108];
  static const int8_T iv0[108] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  real32_T K_k[108];
  static const int8_T iv1[108] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  real32_T fv0[81];
  real32_T c_P_apriori[36];
  static const int8_T iv2[36] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real32_T fv1[36];
  static const int8_T iv3[36] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real32_T S_k[36];
  real32_T d_P_apriori[72];
  static const int8_T iv4[72] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0 };

  real32_T b_K_k[72];
  static const int8_T iv5[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0 };

  real32_T b_r[6];
  static const int8_T iv6[72] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1 };

  static const int8_T iv7[72] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1 };

  real32_T fv2[6];
  real32_T b_z[6];

  /*  Extended Attitude Kalmanfilter */
  /*  */
  /*  state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
  /*  measurement vector z has the following entries [ax,ay,az||mx,my,mz||wmx,wmy,wmz]' */
  /*  knownConst has the following entries [PrvaA,PrvarM,PrvarWO,PrvarW||MsvarA,MsvarM,MsvarW] */
  /*  */
  /*  [x_aposteriori,P_aposteriori] = AttKalman(dt,z_k,x_aposteriori_k,P_aposteriori_k,knownConst) */
  /*  */
  /*  Example.... */
  /*  */
  /*  $Author: Tobias Naegeli $    $Date: 2012 $    $Revision: 1 $ */
  /* coder.varsize('udpIndVect', [9,1], [1,0]) */
  /* udpIndVect=find(updVect); */
  /* process and measurement noise covariance matrix */
  /* 'attitudeKalmanfilter:27' Q = diag(q.^2*dt); */
  power(q, a);
  for (i = 0; i < 12; i++) {
    b_a[i] = a[i] * dt;
  }

  diag(b_a, Q);

  /* observation matrix */
  /* 'attitudeKalmanfilter:37' wx=  x_aposteriori_k(1); */
  /* 'attitudeKalmanfilter:38' wy=  x_aposteriori_k(2); */
  /* 'attitudeKalmanfilter:39' wz=  x_aposteriori_k(3); */
  /* 'attitudeKalmanfilter:41' wox=  x_aposteriori_k(4); */
  /* 'attitudeKalmanfilter:42' woy=  x_aposteriori_k(5); */
  /* 'attitudeKalmanfilter:43' woz=  x_aposteriori_k(6); */
  /* 'attitudeKalmanfilter:45' zex=  x_aposteriori_k(7); */
  /* 'attitudeKalmanfilter:46' zey=  x_aposteriori_k(8); */
  /* 'attitudeKalmanfilter:47' zez=  x_aposteriori_k(9); */
  /* 'attitudeKalmanfilter:49' mux=  x_aposteriori_k(10); */
  /* 'attitudeKalmanfilter:50' muy=  x_aposteriori_k(11); */
  /* 'attitudeKalmanfilter:51' muz=  x_aposteriori_k(12); */
  /* 'attitudeKalmanfilter:54' wk =[wx; */
  /* 'attitudeKalmanfilter:55'      wy; */
  /* 'attitudeKalmanfilter:56'      wz]; */
  /* 'attitudeKalmanfilter:58' wok =[wox;woy;woz]; */
  /* 'attitudeKalmanfilter:59' O=[0,-wz,wy;wz,0,-wx;-wy,wx,0]'; */
  O[0] = 0.0F;
  O[1] = -x_aposteriori_k[2];
  O[2] = x_aposteriori_k[1];
  O[3] = x_aposteriori_k[2];
  O[4] = 0.0F;
  O[5] = -x_aposteriori_k[0];
  O[6] = -x_aposteriori_k[1];
  O[7] = x_aposteriori_k[0];
  O[8] = 0.0F;

  /* 'attitudeKalmanfilter:60' zek =(eye(3)+O*dt)*[zex;zey;zez]; */
  eye(dv0);
  for (i = 0; i < 9; i++) {
    c_a[i] = (real32_T)dv0[i] + O[i] * dt;
  }

  /* 'attitudeKalmanfilter:61' muk =(eye(3)+O*dt)*[mux;muy;muz]; */
  eye(dv0);
  for (i = 0; i < 9; i++) {
    d_a[i] = (real32_T)dv0[i] + O[i] * dt;
  }

  /* 'attitudeKalmanfilter:63' EZ=[0,zez,-zey; */
  /* 'attitudeKalmanfilter:64'     -zez,0,zex; */
  /* 'attitudeKalmanfilter:65'     zey,-zex,0]'; */
  /* 'attitudeKalmanfilter:66' MA=[0,muz,-muy; */
  /* 'attitudeKalmanfilter:67'     -muz,0,mux; */
  /* 'attitudeKalmanfilter:68'     zey,-mux,0]'; */
  /* 'attitudeKalmanfilter:72' E=eye(3); */
  /* 'attitudeKalmanfilter:73' Es=[1,0,0; */
  /* 'attitudeKalmanfilter:74'     0,1,0; */
  /* 'attitudeKalmanfilter:75'     0,0,0]; */
  /* 'attitudeKalmanfilter:76' Z=zeros(3); */
  /* 'attitudeKalmanfilter:77' x_apriori=[wk;wok;zek;muk]; */
  x_n_b[0] = x_aposteriori_k[6];
  x_n_b[1] = x_aposteriori_k[7];
  x_n_b[2] = x_aposteriori_k[8];
  b_x_aposteriori_k[0] = x_aposteriori_k[9];
  b_x_aposteriori_k[1] = x_aposteriori_k[10];
  b_x_aposteriori_k[2] = x_aposteriori_k[11];
  x_apriori[0] = x_aposteriori_k[0];
  x_apriori[1] = x_aposteriori_k[1];
  x_apriori[2] = x_aposteriori_k[2];
  x_apriori[3] = x_aposteriori_k[3];
  x_apriori[4] = x_aposteriori_k[4];
  x_apriori[5] = x_aposteriori_k[5];
  for (i = 0; i < 3; i++) {
    m_n_b[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      m_n_b[i] += c_a[i + 3 * i0] * x_n_b[i0];
    }

    z_n_b[i] = 0.0F;
    for (i0 = 0; i0 < 3; i0++) {
      z_n_b[i] += d_a[i + 3 * i0] * b_x_aposteriori_k[i0];
    }

    x_apriori[i + 6] = m_n_b[i];
  }

  for (i = 0; i < 3; i++) {
    x_apriori[i + 9] = z_n_b[i];
  }

  /* 'attitudeKalmanfilter:79' A_lin=[ Z,  Z,  Z,  Z */
  /* 'attitudeKalmanfilter:80'         Z,  Z,  Z,  Z */
  /* 'attitudeKalmanfilter:81'         EZ, Z,  O,  Z */
  /* 'attitudeKalmanfilter:82'         MA, Z,  Z,  O]; */
  /* 'attitudeKalmanfilter:85' A_lin=eye(12)+A_lin*dt; */
  b_eye(dv1);
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[i0 + 12 * i] = 0.0F;
    }

    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * i) + 3] = 0.0F;
    }
  }

  A_lin[6] = 0.0F;
  A_lin[7] = x_aposteriori_k[8];
  A_lin[8] = -x_aposteriori_k[7];
  A_lin[18] = -x_aposteriori_k[8];
  A_lin[19] = 0.0F;
  A_lin[20] = x_aposteriori_k[6];
  A_lin[30] = x_aposteriori_k[7];
  A_lin[31] = -x_aposteriori_k[6];
  A_lin[32] = 0.0F;
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 3)) + 6] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 6)) + 6] = O[i0 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 9)) + 6] = 0.0F;
    }
  }

  A_lin[9] = 0.0F;
  A_lin[10] = x_aposteriori_k[11];
  A_lin[11] = -x_aposteriori_k[10];
  A_lin[21] = -x_aposteriori_k[11];
  A_lin[22] = 0.0F;
  A_lin[23] = x_aposteriori_k[9];
  A_lin[33] = x_aposteriori_k[7];
  A_lin[34] = -x_aposteriori_k[9];
  A_lin[35] = 0.0F;
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 3)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 6)) + 9] = 0.0F;
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      A_lin[(i0 + 12 * (i + 9)) + 9] = O[i0 + 3 * i];
    }
  }

  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      b_A_lin[i0 + 12 * i] = (real32_T)dv1[i0 + 12 * i] + A_lin[i0 + 12 * i] *
        dt;
    }
  }

  /* 'attitudeKalmanfilter:91' P_apriori=A_lin*P_aposteriori_k*A_lin'+Q; */
  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      A_lin[i + 12 * i0] = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        A_lin[i + 12 * i0] += b_A_lin[i + 12 * i1] * P_aposteriori_k[i1 + 12 *
          i0];
      }
    }
  }

  for (i = 0; i < 12; i++) {
    for (i0 = 0; i0 < 12; i0++) {
      f0 = 0.0F;
      for (i1 = 0; i1 < 12; i1++) {
        f0 += A_lin[i + 12 * i1] * b_A_lin[i0 + 12 * i1];
      }

      P_apriori[i + 12 * i0] = f0 + Q[i + 12 * i0];
    }
  }

  /* %update */
  /* 'attitudeKalmanfilter:95' if updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==1 */
  if ((updateVect[0] == 1) && (updateVect[1] == 1) && (updateVect[2] == 1)) {
    /* 'attitudeKalmanfilter:96' R=diag(r); */
    b_diag(r, R);

    /* observation matrix */
    /* 'attitudeKalmanfilter:99' H_k=[  E,     Es,      Z,    Z; */
    /* 'attitudeKalmanfilter:100'         Z,     Z,      E,    Z; */
    /* 'attitudeKalmanfilter:101'         Z,     Z,      Z,    E]; */
    /* 'attitudeKalmanfilter:103' y_k=z(1:9)-H_k*x_apriori; */
    /* 'attitudeKalmanfilter:105' S_k=H_k*P_apriori*H_k'+R; */
    /* 'attitudeKalmanfilter:106' K_k=(P_apriori*H_k'/(S_k)); */
    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        b_P_apriori[i + 12 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          b_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)iv0[i1
            + 12 * i0];
        }
      }
    }

    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        K_k[i + 9 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          K_k[i + 9 * i0] += (real32_T)iv1[i + 9 * i1] * P_apriori[i1 + 12 * i0];
        }
      }
    }

    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        f0 = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          f0 += K_k[i + 9 * i1] * (real32_T)iv0[i1 + 12 * i0];
        }

        fv0[i + 9 * i0] = f0 + R[i + 9 * i0];
      }
    }

    mrdivide(b_P_apriori, fv0, K_k);

    /* 'attitudeKalmanfilter:109' x_aposteriori=x_apriori+K_k*y_k; */
    for (i = 0; i < 9; i++) {
      f0 = 0.0F;
      for (i0 = 0; i0 < 12; i0++) {
        f0 += (real32_T)iv1[i + 9 * i0] * x_apriori[i0];
      }

      c_a[i] = z[i] - f0;
    }

    for (i = 0; i < 12; i++) {
      f0 = 0.0F;
      for (i0 = 0; i0 < 9; i0++) {
        f0 += K_k[i + 12 * i0] * c_a[i0];
      }

      x_aposteriori[i] = x_apriori[i] + f0;
    }

    /* 'attitudeKalmanfilter:110' P_aposteriori=(eye(12)-K_k*H_k)*P_apriori; */
    b_eye(dv1);
    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        f0 = 0.0F;
        for (i1 = 0; i1 < 9; i1++) {
          f0 += K_k[i + 12 * i1] * (real32_T)iv1[i1 + 9 * i0];
        }

        Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
      }
    }

    for (i = 0; i < 12; i++) {
      for (i0 = 0; i0 < 12; i0++) {
        P_aposteriori[i + 12 * i0] = 0.0F;
        for (i1 = 0; i1 < 12; i1++) {
          P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12 * i0];
        }
      }
    }
  } else {
    /* 'attitudeKalmanfilter:111' else */
    /* 'attitudeKalmanfilter:112' if updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==0 */
    if ((updateVect[0] == 1) && (updateVect[1] == 0) && (updateVect[2] == 0)) {
      /* 'attitudeKalmanfilter:113' R=diag(r(1:3)); */
      c_diag(*(real32_T (*)[3])&r[0], O);

      /* observation matrix */
      /* 'attitudeKalmanfilter:116' H_k=[  E,     Es,      Z,    Z]; */
      /* 'attitudeKalmanfilter:118' y_k=z(1:3)-H_k(1:3,1:12)*x_apriori; */
      /* 'attitudeKalmanfilter:120' S_k=H_k(1:3,1:12)*P_apriori*H_k(1:3,1:12)'+R(1:3,1:3); */
      /* 'attitudeKalmanfilter:121' K_k=(P_apriori*H_k(1:3,1:12)'/(S_k)); */
      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 3; i0++) {
          c_P_apriori[i + 12 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            c_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
              iv2[i1 + 12 * i0];
          }
        }
      }

      for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          fv1[i + 3 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            fv1[i + 3 * i0] += (real32_T)iv3[i + 3 * i1] * P_apriori[i1 + 12 *
              i0];
          }
        }
      }

      for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 3; i0++) {
          f0 = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            f0 += fv1[i + 3 * i1] * (real32_T)iv2[i1 + 12 * i0];
          }

          c_a[i + 3 * i0] = f0 + O[i + 3 * i0];
        }
      }

      b_mrdivide(c_P_apriori, c_a, S_k);

      /* 'attitudeKalmanfilter:124' x_aposteriori=x_apriori+K_k*y_k; */
      for (i = 0; i < 3; i++) {
        f0 = 0.0F;
        for (i0 = 0; i0 < 12; i0++) {
          f0 += (real32_T)iv3[i + 3 * i0] * x_apriori[i0];
        }

        x_n_b[i] = z[i] - f0;
      }

      for (i = 0; i < 12; i++) {
        f0 = 0.0F;
        for (i0 = 0; i0 < 3; i0++) {
          f0 += S_k[i + 12 * i0] * x_n_b[i0];
        }

        x_aposteriori[i] = x_apriori[i] + f0;
      }

      /* 'attitudeKalmanfilter:125' P_aposteriori=(eye(12)-K_k*H_k(1:3,1:12))*P_apriori; */
      b_eye(dv1);
      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          f0 = 0.0F;
          for (i1 = 0; i1 < 3; i1++) {
            f0 += S_k[i + 12 * i1] * (real32_T)iv3[i1 + 3 * i0];
          }

          Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
        }
      }

      for (i = 0; i < 12; i++) {
        for (i0 = 0; i0 < 12; i0++) {
          P_aposteriori[i + 12 * i0] = 0.0F;
          for (i1 = 0; i1 < 12; i1++) {
            P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12 *
              i0];
          }
        }
      }
    } else {
      /* 'attitudeKalmanfilter:126' else */
      /* 'attitudeKalmanfilter:127' if  updateVect(1)==1&&updateVect(2)==1&&updateVect(3)==0 */
      if ((updateVect[0] == 1) && (updateVect[1] == 1) && (updateVect[2] == 0))
      {
        /* 'attitudeKalmanfilter:128' R=diag(r(1:6)); */
        d_diag(*(real32_T (*)[6])&r[0], S_k);

        /* observation matrix */
        /* 'attitudeKalmanfilter:131' H_k=[  E,     Es,      Z,    Z; */
        /* 'attitudeKalmanfilter:132'                 Z,     Z,      E,    Z]; */
        /* 'attitudeKalmanfilter:134' y_k=z(1:6)-H_k(1:6,1:12)*x_apriori; */
        /* 'attitudeKalmanfilter:136' S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6); */
        /* 'attitudeKalmanfilter:137' K_k=(P_apriori*H_k(1:6,1:12)'/(S_k)); */
        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 6; i0++) {
            d_P_apriori[i + 12 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              d_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
                iv4[i1 + 12 * i0];
            }
          }
        }

        for (i = 0; i < 6; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            b_K_k[i + 6 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              b_K_k[i + 6 * i0] += (real32_T)iv5[i + 6 * i1] * P_apriori[i1 + 12
                * i0];
            }
          }
        }

        for (i = 0; i < 6; i++) {
          for (i0 = 0; i0 < 6; i0++) {
            f0 = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              f0 += b_K_k[i + 6 * i1] * (real32_T)iv4[i1 + 12 * i0];
            }

            fv1[i + 6 * i0] = f0 + S_k[i + 6 * i0];
          }
        }

        c_mrdivide(d_P_apriori, fv1, b_K_k);

        /* 'attitudeKalmanfilter:140' x_aposteriori=x_apriori+K_k*y_k; */
        for (i = 0; i < 6; i++) {
          f0 = 0.0F;
          for (i0 = 0; i0 < 12; i0++) {
            f0 += (real32_T)iv5[i + 6 * i0] * x_apriori[i0];
          }

          b_r[i] = z[i] - f0;
        }

        for (i = 0; i < 12; i++) {
          f0 = 0.0F;
          for (i0 = 0; i0 < 6; i0++) {
            f0 += b_K_k[i + 12 * i0] * b_r[i0];
          }

          x_aposteriori[i] = x_apriori[i] + f0;
        }

        /* 'attitudeKalmanfilter:141' P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori; */
        b_eye(dv1);
        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            f0 = 0.0F;
            for (i1 = 0; i1 < 6; i1++) {
              f0 += b_K_k[i + 12 * i1] * (real32_T)iv5[i1 + 6 * i0];
            }

            Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
          }
        }

        for (i = 0; i < 12; i++) {
          for (i0 = 0; i0 < 12; i0++) {
            P_aposteriori[i + 12 * i0] = 0.0F;
            for (i1 = 0; i1 < 12; i1++) {
              P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12 *
                i0];
            }
          }
        }
      } else {
        /* 'attitudeKalmanfilter:142' else */
        /* 'attitudeKalmanfilter:143' if  updateVect(1)==1&&updateVect(2)==0&&updateVect(3)==1 */
        if ((updateVect[0] == 1) && (updateVect[1] == 0) && (updateVect[2] == 1))
        {
          /* 'attitudeKalmanfilter:144' R=diag([r(1:3);r(7:9)]); */
          /* observation matrix */
          /* 'attitudeKalmanfilter:147' H_k=[  E,     Es,      Z,    Z; */
          /* 'attitudeKalmanfilter:148'                     Z,     Z,      Z,    E]; */
          /* 'attitudeKalmanfilter:150' y_k=[z(1:3);z(7:9)]-H_k(1:6,1:12)*x_apriori; */
          /* 'attitudeKalmanfilter:152' S_k=H_k(1:6,1:12)*P_apriori*H_k(1:6,1:12)'+R(1:6,1:6); */
          for (i = 0; i < 6; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              b_K_k[i + 6 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                b_K_k[i + 6 * i0] += (real32_T)iv6[i + 6 * i1] * P_apriori[i1 +
                  12 * i0];
              }
            }
          }

          for (i = 0; i < 3; i++) {
            b_r[i << 1] = r[i];
            b_r[1 + (i << 1)] = r[6 + i];
          }

          for (i = 0; i < 6; i++) {
            for (i0 = 0; i0 < 6; i0++) {
              f0 = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                f0 += b_K_k[i + 6 * i1] * (real32_T)iv7[i1 + 12 * i0];
              }

              S_k[i + 6 * i0] = f0 + b_r[3 * (i + i0)];
            }
          }

          /* 'attitudeKalmanfilter:153' K_k=(P_apriori*H_k(1:6,1:12)'/(S_k)); */
          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 6; i0++) {
              d_P_apriori[i + 12 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                d_P_apriori[i + 12 * i0] += P_apriori[i + 12 * i1] * (real32_T)
                  iv7[i1 + 12 * i0];
              }
            }
          }

          c_mrdivide(d_P_apriori, S_k, b_K_k);

          /* 'attitudeKalmanfilter:156' x_aposteriori=x_apriori+K_k*y_k; */
          for (i = 0; i < 3; i++) {
            b_r[i] = z[i];
          }

          for (i = 0; i < 3; i++) {
            b_r[i + 3] = z[i + 6];
          }

          for (i = 0; i < 6; i++) {
            fv2[i] = 0.0F;
            for (i0 = 0; i0 < 12; i0++) {
              fv2[i] += (real32_T)iv6[i + 6 * i0] * x_apriori[i0];
            }

            b_z[i] = b_r[i] - fv2[i];
          }

          for (i = 0; i < 12; i++) {
            f0 = 0.0F;
            for (i0 = 0; i0 < 6; i0++) {
              f0 += b_K_k[i + 12 * i0] * b_z[i0];
            }

            x_aposteriori[i] = x_apriori[i] + f0;
          }

          /* 'attitudeKalmanfilter:157' P_aposteriori=(eye(12)-K_k*H_k(1:6,1:12))*P_apriori; */
          b_eye(dv1);
          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              f0 = 0.0F;
              for (i1 = 0; i1 < 6; i1++) {
                f0 += b_K_k[i + 12 * i1] * (real32_T)iv6[i1 + 6 * i0];
              }

              Q[i + 12 * i0] = (real32_T)dv1[i + 12 * i0] - f0;
            }
          }

          for (i = 0; i < 12; i++) {
            for (i0 = 0; i0 < 12; i0++) {
              P_aposteriori[i + 12 * i0] = 0.0F;
              for (i1 = 0; i1 < 12; i1++) {
                P_aposteriori[i + 12 * i0] += Q[i + 12 * i1] * P_apriori[i1 + 12
                  * i0];
              }
            }
          }
        } else {
          /* 'attitudeKalmanfilter:158' else */
          /* 'attitudeKalmanfilter:159' x_aposteriori=x_apriori; */
          for (i = 0; i < 12; i++) {
            x_aposteriori[i] = x_apriori[i];
          }

          /* 'attitudeKalmanfilter:160' P_aposteriori=P_apriori; */
          memcpy(&P_aposteriori[0], &P_apriori[0], 144U * sizeof(real32_T));
        }
      }
    }
  }

  /* % euler anglels extraction */
  /* 'attitudeKalmanfilter:169' z_n_b = -x_aposteriori(7:9)./norm(x_aposteriori(7:9)); */
  for (i = 0; i < 3; i++) {
    x_n_b[i] = -x_aposteriori[i + 6];
  }

  rdivide(x_n_b, norm(*(real32_T (*)[3])&x_aposteriori[6]), z_n_b);

  /* 'attitudeKalmanfilter:170' m_n_b = x_aposteriori(10:12)./norm(x_aposteriori(10:12)); */
  rdivide(*(real32_T (*)[3])&x_aposteriori[9], norm(*(real32_T (*)[3])&
           x_aposteriori[9]), m_n_b);

  /* 'attitudeKalmanfilter:172' y_n_b=cross(z_n_b,m_n_b); */
  for (i = 0; i < 3; i++) {
    x_n_b[i] = m_n_b[i];
  }

  cross(z_n_b, x_n_b, m_n_b);

  /* 'attitudeKalmanfilter:173' y_n_b=y_n_b./norm(y_n_b); */
  for (i = 0; i < 3; i++) {
    x_n_b[i] = m_n_b[i];
  }

  rdivide(x_n_b, norm(m_n_b), m_n_b);

  /* 'attitudeKalmanfilter:175' x_n_b=(cross(y_n_b,z_n_b)); */
  cross(m_n_b, z_n_b, x_n_b);

  /* 'attitudeKalmanfilter:176' x_n_b=x_n_b./norm(x_n_b); */
  for (i = 0; i < 3; i++) {
    b_x_aposteriori_k[i] = x_n_b[i];
  }

  rdivide(b_x_aposteriori_k, norm(x_n_b), x_n_b);

  /* 'attitudeKalmanfilter:182' Rot_matrix=[x_n_b,y_n_b,z_n_b]; */
  for (i = 0; i < 3; i++) {
    Rot_matrix[i] = x_n_b[i];
    Rot_matrix[3 + i] = m_n_b[i];
    Rot_matrix[6 + i] = z_n_b[i];
  }

  /* 'attitudeKalmanfilter:186' phi=atan2(Rot_matrix(2,3),Rot_matrix(3,3)); */
  /* 'attitudeKalmanfilter:187' theta=-asin(Rot_matrix(1,3)); */
  /* 'attitudeKalmanfilter:188' psi=atan2(Rot_matrix(1,2),Rot_matrix(1,1)); */
  /* 'attitudeKalmanfilter:189' eulerAngles=[phi;theta;psi]; */
  eulerAngles[0] = rt_atan2f_snf(Rot_matrix[7], Rot_matrix[8]);
  eulerAngles[1] = -(real32_T)asin(Rot_matrix[6]);
  eulerAngles[2] = rt_atan2f_snf(Rot_matrix[3], Rot_matrix[0]);
}

/* End of code generation (attitudeKalmanfilter.c) */

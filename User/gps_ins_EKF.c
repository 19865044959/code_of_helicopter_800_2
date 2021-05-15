#include "gps_ins_EKF.h"

//**************EKF + complementary filter*****************//
STORE_IMU_BUFFER storeIMU;
STORE_GPS_BUFFER storeGPS;
STORE_BAR_BUFFER storeBAR; 
STORE_OUTPUT_BUFFER storeOUTPUT;
IMU_RING_ELEMENT imu_data_new,imu_data_delay;
GPS_RING_ELEMENT gps_data_new,gps_data_delay;
BAR_RING_ELEMENT bar_data_new,bar_data_delay;
OUTPUT_RING_ELEMENT output_data_new,output_data_delay;
double Pos_error_inter[3],Vel_error_inter[3];
bool horizon_success_flag = false;
bool bar_success_flag = false;
bool ekf_update_flag = false;
double Pos_err[3],Vel_err[3],dv1[3],dv2[3] = {0,0,9.84};
double Pos_cor[3],Vel_cor[3];
//**************EKF + complementary filter*****************//

/*********EKF GPS/INS 变量**********/
double pp_init[9] = {0.4,0.4,0.4,0.1,0.1,0.1,0.0097803,0.0097803,0.0097803};
double PP_GI[81];
double F_GI[81];
double G_GI[54];
double R_GI[25];
double A_G_I[81];
double b_A_G_I[81];
double b_G_G_I[54];
double c_A_G_I[81];
double c_G_G_I[81];
static const double b_b[36] = { 0.16, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0025 };

double PP_G_I[81];
double K_GI[45];
static const int d_b_GI[45] = { 6378245, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6378245, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 1, 0, 0, 0, 0 };

double c_a_G_I[25];
double d_a_GI[45];
static const int e_a_GI[45] = { 6378245, 0, 0, 0, 0, 0, 6378245, 0, 0, 0, 0, 0, 1,
	0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0 };
double b_P_GI[81];
/*********EKF GPS/INS 变量**********/

//注：程序详情请看matlab代码
/*****************GPS_INS_EKF Function*******************/
void cross(const double a[3], const double b[3], double c[3])
{
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

void eye(double I[81])
{
  int k;
  memset(&I[0], 0, 81U * sizeof(double));
  for (k = 0; k < 9; k++) {
    I[k + 9 * k] = 1.0;
  }
}

void b_diag(const double v[3], double d[9])
{
  int j;
  memset(&d[0], 0, 9U * sizeof(double));
  for (j = 0; j < 3; j++) {
    d[j + 3 * j] = v[j];
  }
}

void diag(const double v[5], double d[25])
{
  int j;
  memset(&d[0], 0, 25U * sizeof(double));
  for (j = 0; j < 5; j++) {
    d[j + 5 * j] = v[j];
  }
}

void mrdivide(double A[45], const double B[25])
{
  double b_A[25];
  signed char ipiv[5];
  int k;
  int j;
  int c;
  int kBcol;
  int ix;
  double temp;
  double s;
  int i;
  int jp;
  int jAcol;
  memcpy(&b_A[0], &B[0], 25U * sizeof(double));
  for (k = 0; k < 5; k++) {
    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 4; j++) {
    c = j * 6;
    kBcol = 0;
    ix = c;
    temp = fabs(b_A[c]);
    for (k = 2; k <= 5 - j; k++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        kBcol = k - 1;
        temp = s;
      }
    }

    if (b_A[c + kBcol] != 0.0) {
      if (kBcol != 0) {
        ipiv[j] = (signed char)((j + kBcol) + 1);
        ix = j;
        kBcol += j;
        for (k = 0; k < 5; k++) {
          temp = b_A[ix];
          b_A[ix] = b_A[kBcol];
          b_A[kBcol] = temp;
          ix += 5;
          kBcol += 5;
        }
      }

      k = (c - j) + 5;
      for (i = c + 1; i + 1 <= k; i++) {
        b_A[i] /= b_A[c];
      }
    }

    jp = c;
    jAcol = c + 5;
    for (kBcol = 1; kBcol <= 4 - j; kBcol++) {
      temp = b_A[jAcol];
      if (b_A[jAcol] != 0.0) {
        ix = c + 1;
        k = (jp - j) + 10;
        for (i = 6 + jp; i + 1 <= k; i++) {
          b_A[i] += b_A[ix] * -temp;
          ix++;
        }
      }

      jAcol += 5;
      jp += 5;
    }
  }

  for (j = 0; j < 5; j++) {
    jp = 9 * j;
    jAcol = 5 * j;
    for (k = 1; k <= j; k++) {
      kBcol = 9 * (k - 1);
      if (b_A[(k + jAcol) - 1] != 0.0) {
        for (i = 0; i < 9; i++) {
          A[i + jp] -= b_A[(k + jAcol) - 1] * A[i + kBcol];
        }
      }
    }

    temp = 1.0 / b_A[j + jAcol];
    for (i = 0; i < 9; i++) {
      A[i + jp] *= temp;
    }
  }

  for (j = 4; j >= 0; j += -1) {
    jp = 9 * j;
    jAcol = 5 * j - 1;
    for (k = j + 2; k < 6; k++) {
      kBcol = 9 * (k - 1);
      if (b_A[k + jAcol] != 0.0) {
        for (i = 0; i < 9; i++) {
          A[i + jp] -= b_A[k + jAcol] * A[i + kBcol];
        }
      }
    }
  }

  for (kBcol = 3; kBcol >= 0; kBcol += -1) {
    if (ipiv[kBcol] != kBcol + 1) {
      jp = ipiv[kBcol] - 1;
      for (jAcol = 0; jAcol < 9; jAcol++) {
        temp = A[jAcol + 9 * kBcol];
        A[jAcol + 9 * kBcol] = A[jAcol + 9 * jp];
        A[jAcol + 9 * jp] = temp;
      }
    }
  }
}

void kalman_GPS_INS_pv_only_delay(const double V_k[3], const double P_k[3],
  const double acc_bias_k[3], double gps_data[5], const double Fb[3], const
  double T[9], const double pqr[3], double tao, const double PP0[81], double
  Gps_update, double V_n[3], double P_n[3], double P[81], double acc_bias_n[3])
{

  static const double dv0[5] = { 0.09, 0.09, 0.16, 0.0025, 0.0025};

  double x;
  double Rm;
  double Rn;
  double wet[3];
  double we[3];
  double b_Fb[3];
  int i;
  double dv1[3];
  double b_T[3];
  double V_p[3];
  int i0;
  static const double dv2[3] = { 0.0, 0.0, 9.84 };

  double PV_p[6];
  double a;
  double c_T[9];
  static const signed char b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  int i1;
  double dv3[3];
  double b_a[9];
  static const double c_b[3] = { -0.9, 0.0, -0.15 };//GPS安装位置偏差
  double dv4[9];
  double dv5[9];	
  double z[5];
  double XX[9];


  /* GPS/INS?????? ????? */
  /* ???????? */
  /* dx = F_GI*x + G_GI*w             %z = H*x + v */
  /* ???????? */
  /* x(k+1) = A_G_I*x(k) + B*w(k)   %z(k+1) = C*x(k+1) + v(k+1) */
  /* Dpv     ????????, ??????? */
  /* Q      ?????? */
  /* R_GI      ?????? */
  /* Ta     ?????????? */
  /* Tg     ??????????? */
  /* tao    ???? */
  /*     %% ?????? */
  /* ????? */
  /* ???? */
  /* ???????         */
  /*    %% Q??R_GI? */
  /* ????????0.5???? */
  /* ??????????0.5e-4*G_GI   */
  /* ?????? */
  /* %%????????,??? */
  /* %%???????,?? ? */
  /* %%???????,?? ?/? */
  /*  */
  diag(dv0, R_GI);

  /*     %% ?????????,??,???? */
  /*    %% ????????????? */
  /*   */
  /*     %% ?????????????????? */
  x = sin(P_k[0]);
  Rm = 6.378245E+6 * (0.99329437364420614 + 0.010058439533690743 * (x * x));
  x = sin(P_k[0]);
  Rn = 6.378245E+6 * (1.0 - 0.0033528131778969143 * (x * x));

  /*     %% ???????????????? */
  wet[0] = V_k[1] / (Rn + P_k[2]);
  wet[1] = -V_k[0] / (Rm + P_k[2]);
  wet[2] = -V_k[1] * tan(P_k[0]) / (Rn + P_k[2]);

  /*     %% ??????????? */
  we[0] = 7.292E-5 * cos(P_k[0]);
  we[1] = 0.0;
  we[2] = -7.292E-5 * sin(P_k[0]);

  /*     %% ??????? */
  /*     %% ???? ?? */
  for (i = 0; i < 3; i++) {
    b_Fb[i] = 2.0 * we[i] + wet[i];
  }

  cross(b_Fb, V_k, dv1);
  for (i = 0; i < 3; i++) {
    b_Fb[i] = Fb[i] - acc_bias_k[i];
  }

  for (i = 0; i < 3; i++) {
    x = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      x += T[i + 3 * i0] * b_Fb[i0];
    }

    b_T[i] = (x - dv1[i]) + dv2[i];
    V_p[i] = V_k[i] + b_T[i] * tao;
  }

  /*     %% ???? ?? */
  PV_p[0] = P_k[0] + V_k[0] / (Rm + P_k[2]) * tao;
  PV_p[1] = P_k[1] + V_k[1] / ((Rn + P_k[2]) * cos(P_k[0])) * tao;
  PV_p[2] = P_k[2] - V_k[2] * tao;
  PV_p[3] = V_p[0];
  PV_p[4] = V_p[1];
  PV_p[5] = V_p[2];

  /*     %% ?????????? */
  memset(&F_GI[0], 0, 81U * sizeof(double));
  x = Rm + P_k[2];
  F_GI[18] = -V_k[0] / (x * x);
  F_GI[27] = 1.0 / (Rm + P_k[2]);
  F_GI[1] = V_k[1] * (1.0 / cos(P_k[0])) * tan(P_k[0]) / (Rn + P_k[2]);
  x = Rn + P_k[2];
  F_GI[19] = -V_k[1] * (1.0 / cos(P_k[0])) / (x * x);
  F_GI[37] = 1.0 / cos(P_k[0]) / (Rn + P_k[2]);
  F_GI[47] = -1.0;
  x = 1.0 / cos(P_k[0]);
  F_GI[3] = -0.00014584 * V_k[1] * cos(P_k[0]) - V_k[1] * V_k[1] * (x * x) / (Rn +
    P_k[2]);
  x = Rn + P_k[2];
  a = Rm + P_k[2];
  F_GI[21] = V_k[1] * V_k[1] * tan(P_k[0]) / (x * x) - V_k[0] * V_k[2] / (a * a);
  F_GI[30] = V_k[2] / (Rm + P_k[2]);
  F_GI[39] = -0.00014584 * sin(P_k[0]) - 2.0 * V_k[1] * tan(P_k[0]) / (Rn + P_k[2]);
  F_GI[48] = V_k[0] / (Rm + P_k[2]);
  x = 1.0 / cos(P_k[0]);
  F_GI[4] = (0.00014584 * V_k[0] * cos(P_k[0]) + V_k[1] * V_k[0] * (x * x) / (Rn +
           P_k[2])) - 0.00014584 * V_k[2] * sin(P_k[0]);
  x = Rn + P_k[2];
  a = Rn + P_k[2];
  F_GI[22] = -V_k[0] * V_k[1] * tan(P_k[0]) / (x * x) - V_k[1] * V_k[2] / (a * a);
  F_GI[31] = 0.00014584 * sin(P_k[0]) + V_k[1] * tan(P_k[0]) / (Rn + P_k[2]);
  F_GI[40] = V_k[2] / (Rn + P_k[2]) + V_k[0] * tan(P_k[0]) / (Rn + P_k[2]);
  F_GI[49] = 0.00014584 * cos(P_k[0]) + V_k[1] / (Rn + P_k[2]);
  F_GI[5] = 0.00014584 * V_k[1] * sin(P_k[0]);
  x = Rm + P_k[2];
  a = Rn + P_k[2];
  F_GI[23] = V_k[0] * V_k[0] / (x * x) + V_k[1] * V_k[1] / (a * a);
  F_GI[32] = -2.0 * V_k[0] / (Rm + P_k[2]);
  F_GI[41] = -0.00014584 * cos(P_k[0]) - 2.0 * V_k[1] / (Rn + P_k[2]);
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      F_GI[(i0 + 9 * (6 + i)) + 3] = -T[i0 + 3 * i];
    }
  }

  /*     %% G_GI? */
  memset(&G_GI[0], 0, 54U * sizeof(double));
  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      G_GI[(i0 + 9 * i) + 3] = b[i0 + 3 * i];
      c_T[i0 + 3 * i] = -T[i0 + 3 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      G_GI[(i + 9 * (3 + i0)) + 3] = 0.0;
      for (i1 = 0; i1 < 3; i1++) {
        G_GI[(i + 9 * (3 + i0)) + 3] += c_T[i + 3 * i1] * (double)b[i1 + 3 * i0];
      }
    }
  }

  for (i = 0; i < 3; i++) {
    for (i0 = 0; i0 < 3; i0++) {
      G_GI[(i0 + 9 * (3 + i)) + 6] = b[i0 + 3 * i];
    }
  }

  /*     %% ??? */
  eye(A_G_I);
  for (i = 0; i < 81; i++) {
    A_G_I[i] += F_GI[i] * tao;
  }

  for (i = 0; i < 54; i++) {
    G_GI[i] *= tao;
  }

  /*     %% ??????? */
  for (i = 0; i < 9; i++) {
    for (i0 = 0; i0 < 9; i0++) {
      b_A_G_I[i + 9 * i0] = 0.0;
      for (i1 = 0; i1 < 9; i1++) {
        b_A_G_I[i + 9 * i0] += A_G_I[i + 9 * i1] * PP0[i1 + 9 * i0];
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      b_G_G_I[i + 9 * i0] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        b_G_G_I[i + 9 * i0] += G_GI[i + 9 * i1] * b_b[i1 + 6 * i0];
      }
    }

    for (i0 = 0; i0 < 9; i0++) {
      c_A_G_I[i + 9 * i0] = 0.0;
      for (i1 = 0; i1 < 9; i1++) {
        c_A_G_I[i + 9 * i0] += b_A_G_I[i + 9 * i1] * A_G_I[i0 + 9 * i1];
      }

      c_G_G_I[i + 9 * i0] = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        c_G_G_I[i + 9 * i0] += b_G_G_I[i + 9 * i1] * G_GI[i0 + 9 * i1];
      }
    }
  }

  for (i = 0; i < 9; i++) {
    for (i0 = 0; i0 < 9; i0++) {
      PP_G_I[i0 + 9 * i] = c_A_G_I[i0 + 9 * i] + c_G_G_I[i0 + 9 * i];
    }
  }

  if (Gps_update == 1.0) {
    /*        %% ????? */
    /*        %% ??????? */
    dv3[0] = 1.0 / (Rm + P_k[2]);
    dv3[1] = 1.0 / ((Rn + P_k[2]) * cos(P_k[0]));
    dv3[2] = -1.0;
    b_diag(dv3, c_T);
    for (i = 0; i < 3; i++) {
      b_Fb[i] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        b_a[i + 3 * i0] = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          b_a[i + 3 * i0] += c_T[i + 3 * i1] * T[i1 + 3 * i0];
        }

        b_Fb[i] += b_a[i + 3 * i0] * c_b[i0];
      }
    }

    for (i = 0; i < 2; i++) {
      gps_data[i] -= b_Fb[i];
    }

    for (i = 0; i < 3; i++) {
      we[i] += wet[i];
    }

    dv4[0] = 0.0;
    dv4[3] = -we[2];
    dv4[6] = we[1];
    dv4[1] = we[2];
    dv4[4] = 0.0;
    dv4[7] = -we[0];
    dv4[2] = -we[1];
    dv4[5] = we[0];
    dv4[8] = 0.0;
    dv5[0] = 0.0;
    dv5[3] = -pqr[2];
    dv5[6] = pqr[1];
    dv5[1] = pqr[2];
    dv5[4] = 0.0;
    dv5[7] = -pqr[0];
    dv5[2] = -pqr[1];
    dv5[5] = pqr[0];
    dv5[8] = 0.0;
    for (i = 0; i < 3; i++) {
      b_Fb[i] = 0.0;
      b_T[i] = 0.0;
      for (i0 = 0; i0 < 3; i0++) {
        b_a[i + 3 * i0] = 0.0;
        c_T[i + 3 * i0] = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          b_a[i + 3 * i0] += dv4[i + 3 * i1] * T[i1 + 3 * i0];
          c_T[i + 3 * i0] += T[i + 3 * i1] * dv5[i1 + 3 * i0];
        }

        b_Fb[i] += b_a[i + 3 * i0] * c_b[i0];
        b_T[i] += c_T[i + 3 * i0] * c_b[i0];
      }

      dv1[i] = b_Fb[i] - b_T[i];
    }

    for (i = 0; i < 2; i++) {
      gps_data[3 + i] += dv1[i];
    }

    /*         %% Kalman?? */
    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 5; i0++) {
        K_GI[i + 9 * i0] = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          K_GI[i + 9 * i0] += PP_G_I[i + 9 * i1] * (double)d_b_GI[i1 + 9 * i0];
        }
      }
    }

    for (i = 0; i < 5; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        d_a_GI[i + 5 * i0] = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          d_a_GI[i + 5 * i0] += (double)e_a_GI[i + 5 * i1] * PP_G_I[i1 + 9 * i0];
        }
      }

      for (i0 = 0; i0 < 5; i0++) {
        x = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          x += d_a_GI[i + 5 * i1] * (double)d_b_GI[i1 + 9 * i0];
        }

        c_a_G_I[i + 5 * i0] = x + R_GI[i + 5 * i0];
      }
    }

    mrdivide(K_GI, c_a_G_I);
    eye(F_GI);
    for (i = 0; i < 9; i++) {
      for (i0 = 0; i0 < 9; i0++) {
        x = 0.0;
        for (i1 = 0; i1 < 5; i1++) {
          x += K_GI[i + 9 * i1] * (double)e_a_GI[i1 + 5 * i0];
        }

        A_G_I[i + 9 * i0] = F_GI[i + 9 * i0] - x;
      }

      for (i0 = 0; i0 < 9; i0++) {
        P[i + 9 * i0] = 0.0;
        for (i1 = 0; i1 < 9; i1++) {
          P[i + 9 * i0] += A_G_I[i + 9 * i1] * PP_G_I[i1 + 9 * i0];
        }
      }
    }

    for (i = 0; i < 5; i++) {
      z[i] = PV_p[i] - gps_data[i];
    }

    z[0] *= 6.378245E+6;
    z[1] *= 6.378245E+6;

    /*         %% ????,?? */
    for (i = 0; i < 9; i++) {
      XX[i] = 0.0;
      for (i0 = 0; i0 < 5; i0++) {
        XX[i] += K_GI[i + 9 * i0] * z[i0];
      }
    }

    for (i = 0; i < 3; i++) {
      V_n[i] = PV_p[i + 3] - XX[i + 3];
      P_n[i] = PV_p[i] - XX[i];
      acc_bias_n[i] = acc_bias_k[i] - XX[i + 6];
    }

    /*     */
  } else {
    for (i = 0; i < 3; i++) {
      V_n[i] = PV_p[i + 3];
      P_n[i] = PV_p[i];
      acc_bias_n[i] = acc_bias_k[i];
    }

    memcpy(&P[0], &PP_G_I[0], 81U * sizeof(double));
  }

  for (i = 0; i < 9; i++) {
    for (i0 = 0; i0 < 9; i0++) {
      b_P_GI[i0 + 9 * i] = (P[i0 + 9 * i] + P[i + 9 * i0]) / 2.0;
    }
  }

  for (i = 0; i < 9; i++) {
    memcpy(&P[i * 9], &b_P_GI[i * 9], 9U * sizeof(double));
  }
}


void Imu_Data_Push(IMU_RING_ELEMENT data)
{
	storeIMU.youngest = (storeIMU.youngest + 1)%IMU_BUFFER_SIZE;
	storeIMU.imu_data[storeIMU.youngest] = data;
	storeIMU.oldest = (storeIMU.youngest + 1)%IMU_BUFFER_SIZE;
	if(storeIMU.oldest ==0) storeIMU.is_filled = true;
}

void Imu_Data_Pop(IMU_RING_ELEMENT* data)
{
	*data = storeIMU.imu_data[storeIMU.oldest];
}

void Read_Imu_Data(sAHRS *ele)
{
	u8 index = 0;
	if(ahrs[1].gps_ins_update_flag)
	{
		imu_data_new.update_time = ahrs[1].ahrs_update_time;
		for(u8 i=0;i<3;i++)
		{
			imu_data_new.acc[i] = ele->AccB[i]; 
		}
	
		for(u8 i=0;i<3;i++)
		{
			for(u8 j=0;j<3;j++)
			{
				imu_data_new.T[index] = (double)ahrs[1].Cb2n[j][i];
				index++;
			}
		}
		Imu_Data_Push(imu_data_new);
		Imu_Data_Pop(&imu_data_delay);
		ahrs[1].gps_ins_update_flag = false;
	}
}


void Gps_Data_Push(GPS_RING_ELEMENT data)
{
	storeGPS.youngest = (storeGPS.youngest + 1)%GPS_BUFFER_SIZE;
	storeGPS.gps_data[storeGPS.youngest] = data;
}

void Read_Gps_Data(sGPS* ele)
{
	if(ele->gps_update_time - storeGPS.last_update_time > 70000)
	{
		gps_data_new.Pos_obs[0] = ele->lat*D2R_D; 
		gps_data_new.Pos_obs[1] = ele->lng*D2R_D;
		gps_data_new.Vel_obs[0] = ele->NED_spd[0];
		gps_data_new.Vel_obs[1] = ele->NED_spd[1];
		storeGPS.last_update_time = ele->gps_update_time;
		gps_data_new.update_time = ele->gps_update_time - GPS_DELAYED_TIME - 10000;
		storeGPS.new_data_flag = true;
		Gps_Data_Push(gps_data_new);
	}
}

void Bar_Data_Push(BAR_RING_ELEMENT data)
{
	storeBAR.youngest = (storeBAR.youngest + 1)%IMU_BUFFER_SIZE;
	storeBAR.bar_data[storeBAR.youngest] = data;
	//storeBAR.oldest = (storeBAR.youngest + 1)%IMU_BUFFER_SIZE;
	//storeBAR.oldest = (storeBAR.oldest + 1)%IMU_BUFFER_SIZE;
}

void Bar_Data_Pop(BAR_RING_ELEMENT *data)
{
   *data = storeBAR.bar_data[storeBAR.oldest];
}

//void Read_Baro_Data(sEXIT* ele)
//{
//	if(ele->gps_ins_update_flag)
//	{
//		bar_data_new.update_time = ele->height_update_time - 25000;
//		bar_data_new.Height = exitx->DatRel[2];
//		storeBAR.new_data_flag = true;
//		Bar_Data_Push(bar_data_new);
//		//Bar_Data_Pop(&bar_data_delay);
//		ele->gps_ins_update_flag = false;
//	}
//}
void Read_Baro_Data(sMS5611* ele)
{
	if(ele->gps_ins_update_flag)
	{
		bar_data_new.update_time = ele->height_update_time - 25000;
		bar_data_new.Height = ele->AltFil;
		storeBAR.new_data_flag = true;
		Bar_Data_Push(bar_data_new);
		//Bar_Data_Pop(&bar_data_delay);
		ele->gps_ins_update_flag = false;
	}
}

void Store_Buffer_Init(sGPS *ele)
{
	u8 index = 0;
	storeIMU.oldest = storeIMU.youngest = 0;
	storeIMU.is_filled = false;
	storeGPS.oldest = storeGPS.youngest = 0;
	storeBAR.oldest = storeBAR.youngest = 0;
	storeOUTPUT.oldest = storeOUTPUT.youngest = 0;
	for(u8 i=0;i<3;i++)
	{
		Pos_error_inter[i] = 0;
		Vel_error_inter[i] = 0;
	}
	if(ahrs[1].gps_ins_update_flag)
	{
		imu_data_new.update_time = ahrs[1].ahrs_update_time;;
		for(u8 i=0;i<3;i++)
		{
			imu_data_new.acc[i] = ahrs[1].AccB[i];
			imu_data_new.w_qpr[i] = ahrs[1].pqr[i];
		}
		for(u8 i=0;i<3;i++)
		{
			for(u8 j=0;j<3;j++)
			{
				imu_data_new.T[index] = (double)ahrs[1].Cb2n[j][i];
				index++;
			}
		}
		storeIMU.imu_data[0] = imu_data_new;
		ahrs[1].gps_ins_update_flag = false;
	}
	if(ele->GPSUpdate)
	{
		gps_data_new.Pos_obs[0] = ele->lat*D2R_D; 
		gps_data_new.Pos_obs[1] = ele->lng*D2R_D;
		gps_data_new.Vel_obs[0] = ele->NED_spd[0];
		gps_data_new.Vel_obs[1] = ele->NED_spd[1];
		gps_data_new.update_time = ele->gps_update_time - GPS_DELAYED_TIME - 5000;
		storeGPS.gps_data[0] = gps_data_new;
		ele->GPSUpdate = 0;
		
		output_data_new.Pos[0] = ele->lat*D2R_D;
		output_data_new.Pos[1] = ele->lng*D2R_D;
		output_data_new.Pos[2] = ms5611[0].AltRel; 
		output_data_new.Vel[0] = ele->NED_spd[0];
		output_data_new.Vel[1] = ele->NED_spd[1];
		output_data_new.Vel[2] = 0.0;
		
		for(int i = 0;i<IMU_BUFFER_SIZE;i++)
		{
			storeOUTPUT.output_data[i].Pos[0] = ele->lat*D2R_D;
			storeOUTPUT.output_data[i].Pos[1] = ele->lng*D2R_D;
			storeOUTPUT.output_data[i].Pos[2] = exitx->DatRel[2];
			storeOUTPUT.output_data[i].Vel[0] = ele->NED_spd[0];
			storeOUTPUT.output_data[i].Vel[1] = ele->NED_spd[1];
			storeOUTPUT.output_data[i].Vel[2] = 0.0;
		}
	}
	//if(exitx->gps_ins_update_flag)
	if(ms5611->gps_ins_update_flag)
	{
		//bar_data_new.Height = exitx->DatRel[2];
		//		bar_data_new.update_time = exitx->height_update_time - 15000;
		//	exitx->gps_ins_update_flag = false;
		bar_data_new.Height = ms5611[0].AltFil;
		bar_data_new.update_time = ms5611->height_update_time - 15000;
		ms5611->gps_ins_update_flag = false;
		storeBAR.bar_data[0] = bar_data_new;
	}
}


bool Recall_Gps_data(STORE_GPS_BUFFER* data,u32 sampleing_time)
{
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeGPS.new_data_flag)
		return false;
	if(data->youngest == oldest)
	{
		if((data->gps_data[oldest].update_time!=0) && (data->gps_data[oldest].update_time <= sampleing_time)){
			if(sampleing_time - data->gps_data[oldest].update_time < 100000)
			{
				bestindex = oldest;
				success = true;
				storeGPS.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)
		{
			if((data->gps_data[oldest].update_time!=0) && (data->gps_data[oldest].update_time <= sampleing_time)){
				if(sampleing_time - data->gps_data[oldest].update_time < 100000)
				{
					bestindex = oldest;
					success = true;
				}
			}
			else if (data->gps_data[oldest].update_time > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%GPS_BUFFER_SIZE;
		}
	}
	
	if(success)
	{
		gps_data_delay = data->gps_data[bestindex];
		data->oldest = (bestindex+1)%GPS_BUFFER_SIZE;
		data->gps_data[bestindex].update_time = 0;
		return true;
	}
	else
	{
		return false;
	}
}

bool Recall_Bar_data(STORE_BAR_BUFFER* data,u32 sampleing_time)
{
	u8 oldest = data->oldest;
	u8 bestindex;
	bool success = false;
	if(!storeBAR.new_data_flag)
		return false;
	if(data->youngest == oldest)
	{
		if((data->bar_data[oldest].update_time!=0) && (data->bar_data[oldest].update_time <= sampleing_time)){
			if(sampleing_time - data->bar_data[oldest].update_time < 50000)
			{
				bestindex = oldest;
				success = true;
				storeBAR.new_data_flag = false;
			}
		}
	}
	else{
		while(data->youngest != oldest)
		{
			if((data->bar_data[oldest].update_time!=0) && (data->bar_data[oldest].update_time <= sampleing_time)){
				if(sampleing_time - data->bar_data[oldest].update_time < 50000)
				{
					bestindex = oldest;
					success = true;
				}
			}
			else if (data->bar_data[oldest].update_time > sampleing_time)
			{
				break;
			}
			oldest = (oldest+1)%IMU_BUFFER_SIZE;
		}
	}
	
	if(success)
	{
		bar_data_delay = data->bar_data[bestindex];
		data->oldest = (bestindex+1)%IMU_BUFFER_SIZE;
		data->bar_data[bestindex].update_time = 0;
		return true;
	}
	else
	{
		return false;
	}
}

void Output_Data_Push(OUTPUT_RING_ELEMENT data)
{
	storeOUTPUT.youngest = (storeOUTPUT.youngest + 1)%IMU_BUFFER_SIZE;
	storeOUTPUT.output_data[storeOUTPUT.youngest] = data;
	storeOUTPUT.oldest = (storeOUTPUT.youngest + 1)%IMU_BUFFER_SIZE;
}

void Output_Data_Pop(OUTPUT_RING_ELEMENT* data)
{
	*data = storeOUTPUT.output_data[storeOUTPUT.oldest];
}

void Calculate_Output(sGPS* ele)
{
	double Fb[3],b_T[3],P_temp[3],Vel_last[3];
	double x,Rm,Rn;
	double tao=0.01;
	
  x = sin(output_data_new.Pos[0]);
  Rm = 6.378245E+6 * (0.99329437364420614 + 0.010058439533690743 * (x * x));
  x = sin(output_data_new.Pos[0]);
  Rn = 6.378245E+6 * (1.0 - 0.0033528131778969143 * (x * x));

  Fb[0] = 2.0 * (7.292E-5 * cos(output_data_new.Pos[0])) + output_data_new.Vel[1] / (Rn + output_data_new.Pos[2]);
  Fb[1] = -output_data_new.Vel[0] / (Rm + output_data_new.Pos[2]);
  Fb[2] = 2.0 * (-7.292E-5 * sin(output_data_new.Pos[0])) - output_data_new.Vel[1] * tan(output_data_new.Pos[0]) / (Rn + output_data_new.Pos[2]);
  
	cross(Fb, output_data_new.Vel, dv1);
  
	for (u8 i = 0; i < 3; i++) {
    Fb[i] = imu_data_new.acc[i] - ele->acc_bias_EKF[i];
		Vel_last[i] = output_data_new.Vel[i];
  }

  for (u8 i = 0; i < 3; i++) {
    x = 0.0;
    for (u8 j = 0; j < 3; j++) {
      x += imu_data_new.T[i + 3 * j] * Fb[j];
    }

    b_T[i] = (x - dv1[i]) + dv2[i];
    output_data_new.Vel[i] = output_data_new.Vel[i] + b_T[i] * tao;
  }

  P_temp[0] = output_data_new.Pos[0] + 0.5 * (output_data_new.Vel[0] + Vel_last[0])/ (Rm + output_data_new.Pos[2]) * tao;
  P_temp[1] = output_data_new.Pos[1] + 0.5 * (output_data_new.Vel[1] + Vel_last[1])/ ((Rn + output_data_new.Pos[2]) * cos(output_data_new.Pos[0])) * tao;
  P_temp[2] = output_data_new.Pos[2] - 0.5 * (output_data_new.Vel[2] + Vel_last[2])* tao;
	
	output_data_new.Pos[0] = P_temp[0];
	output_data_new.Pos[1] = P_temp[1];
	output_data_new.Pos[2] = P_temp[2];
	
	Output_Data_Push(output_data_new);
	Output_Data_Pop(&output_data_delay);
	
	for(u8 i = 0;i<3;i++)
	{
		Pos_err[i] = ele->P_EKF[i] - output_data_delay.Pos[i];
		Pos_error_inter[i] += Pos_err[i];
		Vel_err[i] = ele->V_EKF[i] - output_data_delay.Vel[i];
		Vel_error_inter[i] += Vel_err[i];
	}
	double tau = 0.25;
	double PosVelGain = 0.1;
	for(u8 i = 0;i<3;i++)
	{
		Pos_cor[i] = Pos_err[i] * PosVelGain + Pos_error_inter[i] * (PosVelGain * PosVelGain) * 0.1;
	}
	//Pos_cor[2] = Pos_err[2] * 2 * PosVelGain + Pos_error_inter[2] * (2 * PosVelGain * 2 * PosVelGain) * 0.1;
	for(u8 i = 0;i<3;i++)
	{
		Vel_cor[i] = Vel_err[i] * PosVelGain + Vel_error_inter[i] * (PosVelGain * PosVelGain) * 0.1;
	}
	OUTPUT_RING_ELEMENT output_state;
	for(u8 i = 0;i<IMU_BUFFER_SIZE;i++)
	{
		output_state = storeOUTPUT.output_data[i];
		
		for(u8 j=0;j<3;j++)
		{
			output_state.Pos[j] = output_state.Pos[j] + Pos_cor[j];
			output_state.Vel[j] = output_state.Vel[j] + Vel_cor[j];
		}
		storeOUTPUT.output_data[i] = output_state;
	}
	output_data_new = storeOUTPUT.output_data[storeOUTPUT.youngest];
}

void GPS_INS_EKF(sGPS* ele)
{ 
	
	if((ele->star>5)&&(!ele->GPS_INS_EKF_flag)&&(ahrs[1].circle_loop>=400)&&(!ele->GPS_INS_EKF_start_flag)&&ms5611[0].AltOff!=0)//星数要求5   高度初始化完毕
	{
		ele->GPS_INS_EKF_flag = true;
		ele->V_EKF[0] = ele->NED_spd[0];
		ele->V_EKF[1] = ele->NED_spd[1];
		ele->V_EKF[2] = 0.0;
		ele->P_EKF[0] = ele->lat*D2R_D;
		ele->P_EKF[1] = ele->lng*D2R_D;
		//ele->P_EKF[2] = exitx->DatRel[2];
		ele->P_EKF[2] = ms5611[0].AltRel;
		ele->acc_bias_EKF[0] = 0.0;
		ele->acc_bias_EKF[1] = 0.0;
		ele->acc_bias_EKF[2] = 0.0;
		for(int i =1;i<=9;i++)
		{
			ele->P_GI[(i-1)*9 + i-1] = pp_init[i-1];
		}
		//FIFO初始化函数
		Store_Buffer_Init(ele);
	}
	//IMUFIFO 数据满了才开始进行EKF
	if(ele->GPS_INS_EKF_flag)
	{
		Read_Imu_Data(&ahrs[1]);
		Read_Gps_Data(ele);
		//Read_Baro_Data(exitx);
		Read_Baro_Data(ms5611);
		if(storeIMU.is_filled)
		{
			ele->GPS_INS_EKF_start_flag = true;
			ele->GPS_INS_EKF_flag = false;
		}
	}
	//初始化完成后进行9阶EKF
	if(ele->GPS_INS_EKF_start_flag)
	{   
		double gps_data[5];
		Read_Imu_Data(&ahrs[1]);
		Read_Gps_Data(ele);
	 //Read_Baro_Data(exitx);
		Read_Baro_Data(ms5611);
		
		horizon_success_flag = Recall_Gps_data(&storeGPS,imu_data_delay.update_time);
		
		bar_success_flag = Recall_Bar_data(&storeBAR,imu_data_delay.update_time);
		
		ekf_update_flag = horizon_success_flag & 1;   ///bug
		if(ekf_update_flag)
		{		
			static u32 updata_array[100];
			static u8 ekf_update_index = 0;
			updata_array[ekf_update_index++] = imu_data_delay.update_time;
			if(ekf_update_index == 100) ekf_update_index = 0;
			for(int i=0;i<2;i++)
			{
				gps_data[i] = gps_data_delay.Pos_obs[i];
			}
			gps_data[2] = bar_data_delay.Height;
			for(int i=0;i<2;i++)
			{
				gps_data[i+3] = gps_data_delay.Vel_obs[i];
			}
		}
		kalman_GPS_INS_pv_only_delay(ele->V_EKF,ele->P_EKF,ele->acc_bias_EKF,gps_data,imu_data_delay.acc,imu_data_delay.T,imu_data_delay.w_qpr,0.01,ele->P_GI,ekf_update_flag,ele->V_P,ele->P_p,PP_GI,ele->acc_bias_p);
		//下一时刻预测的速度、位置、加速度偏置
		for(int i=0;i<3;i++)
		{
			ele->V_EKF[i] = ele->V_P[i];
			ele->P_EKF[i] = ele->P_p[i];
			ele->acc_bias_EKF[i] = ele->acc_bias_p[i];
		}
		for(int i=0;i<81;i++)
		{
			ele->P_GI[i] = PP_GI[i];
		}
		Calculate_Output(ele);
	}
}
/*****************GPS_INS_EKF Function*******************/

/*******************经纬度转NED坐标系********************/
void LLH2NED(sGPS* ele)
{
	//零点ECEF初始完毕才开始计算NED坐标系相对位置
	if(ele->ECEF_Init_Flag && ele->GPS_INS_EKF_start_flag)
	{
		ele->LLH[0] = output_data_new.Pos[0];//ele->P_EKF[0];         //纬度
		ele->LLH[1] = output_data_new.Pos[1];         //经度
		ele->LLH[2] = output_data_new.Pos[2];//高度，注意是该减还是加
	//	ele->LLH[2] = 0.0f;//高度，注意是该减还是加
		//LLH换算成ECFF（地球中心坐标系）
		ele->N = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(ele->LLH[0])));
		ele->ECFF[0]= (ele->N + ele->LLH[2]) * cos(ele->LLH[0]) * cos(ele->LLH[1]);
		ele->ECFF[1]= (ele->N + ele->LLH[2]) * cos(ele->LLH[0]) * sin(ele->LLH[1]);
		ele->ECFF[2]= (ele->N*(1-SQR(C_WGS84_e)) +ele->LLH[2]) * sin(ele->LLH[0]);
		//减去零点ECEF
		ele->ECFF[0] -= ele->Zero_ECFF[0];
		ele->ECFF[1] -= ele->Zero_ECFF[1];
		ele->ECFF[2] -= ele->Zero_ECFF[2];
		//北东地坐标系坐标
		ele->NED[0] = ele->Re2t[0][0]*ele->ECFF[0]+ele->Re2t[0][1]*ele->ECFF[1]+ele->Re2t[0][2]*ele->ECFF[2];
		ele->NED[1] = ele->Re2t[1][0]*ele->ECFF[0]+ele->Re2t[1][1]*ele->ECFF[1]+ele->Re2t[1][2]*ele->ECFF[2];
		ele->NED[2] = ele->Re2t[2][0]*ele->ECFF[0]+ele->Re2t[2][1]*ele->ECFF[1]+ele->Re2t[2][2]*ele->ECFF[2];
	  ele->NED[2] = output_data_new.Pos[2];
	//	ele->NED[2] = 0.0f;
		
		ele->LLH_Init[0] = ele->lat*D2R_D;         //纬度
		ele->LLH_Init[1] = ele->lng*D2R_D;         //经度
		ele->LLH_Init[2] = ele->P_EKF[2];//高度，注意是该减还是加
		
		//LLH换算成ECFF（地球中心坐标系）
		ele->N_Init = C_WGS84_a/sqrt( 1.0 - SQR(C_WGS84_e)*SQR(sin(ele->LLH_Init[0])));
		ele->ECFF_Init[0]= (ele->N_Init + ele->LLH_Init[2]) * cos(ele->LLH_Init[0]) * cos(ele->LLH_Init[1]);
		ele->ECFF_Init[1]= (ele->N_Init + ele->LLH_Init[2]) * cos(ele->LLH_Init[0]) * sin(ele->LLH_Init[1]);
		ele->ECFF_Init[2]= (ele->N_Init*(1-SQR(C_WGS84_e)) +ele->LLH_Init[2]) * sin(ele->LLH_Init[0]);
	
		ele->ECFF_Init[0] -= ele->Zero_ECFF[0];
		ele->ECFF_Init[1] -= ele->Zero_ECFF[1];
		ele->ECFF_Init[2] -= ele->Zero_ECFF[2];
	
		ele->NED_Init[0] = ele->Re2t[0][0]*ele->ECFF_Init[0]+ele->Re2t[0][1]*ele->ECFF_Init[1]+ele->Re2t[0][2]*ele->ECFF_Init[2];
		ele->NED_Init[1] = ele->Re2t[1][0]*ele->ECFF_Init[0]+ele->Re2t[1][1]*ele->ECFF_Init[1]+ele->Re2t[1][2]*ele->ECFF_Init[2];
		ele->NED_Init[2] = ele->Re2t[2][0]*ele->ECFF_Init[0]+ele->Re2t[2][1]*ele->ECFF_Init[1]+ele->Re2t[2][2]*ele->ECFF_Init[2];
	}
}

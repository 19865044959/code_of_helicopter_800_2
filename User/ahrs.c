#include "ahrs.h"

/***************************************************\
功能：
  惯导系统，欧拉角姿态解算，四元数解析方式
说明：
  1、采用欧拉角表示，四元数计算方法
  2、PI互补滤波
	3、7阶EKF
\***************************************************/

sAHRS ahrs[AHRS_NUM];
/*********EKF 姿态 变量************/
float F[49];
float tao[49];
float c_a[49];
float b_tao[49];
float b_F[49];
float c_tao[49];
float b_a[49];
static const signed char a[49] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
0, 0, 0, 0, 0, 0, 0, 1 };
//Q阵
static const float b[49] = { 0.000194F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
0.000194F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.000194F, 0.0F, 0.0F, 0.0F,
0.0F, 0.0F, 0.0F, 0.0F, 0.000194F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
0.0001F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0001F, 0.0F, 0.0F, 0.0F,
0.0F, 0.0F, 0.0F, 0.0F, 0.0001F };
/*********EKF 姿态 变量************/

/******************功能函数****************/
void Ahrs_Init_Para(sAHRS *ele,sAHRS_CFG *elecfg)
{
	ele->name = elecfg->name;
	ele->Update = false;
	ele->halfDt = elecfg->halfDt;
	for(u8 i=0;i<3;i++)
	{
		ele->pqr[i]=0.0f;
		ele->Ang[i]=0.0f;
		ele->Acc[i] = 0.0f;
		ele->Uvw[i] = 0.0f;
		ele->Xyz[i] = 0.0f;
		ele->AngOff[i]=elecfg->AngOff[i];
	}
	memset(ele->LLS,0,9);
	if(elecfg->mode == FLY_HEL)
	{
		ele->LLS[0][0]=1.0f;
		ele->LLS[1][1]=1.0f;
		ele->LLS[2][2]=1.0f;
	}
	else
	{
		ele->LLS[0][2]=-1.0f;
		ele->LLS[1][1]=1.0f;
		ele->LLS[2][0]=1.0f;
	}
	arm_mat_init_f32(&ele->mat33,3,3,ele->LLS[0]);
	ele->w = 1;
	ele->x = 0;
	ele->y = 0;
	ele->z = 0;
	ele->exInt = 0.0f;
	ele->eyInt = 0.0f;
	ele->ezInt = 0.0f;
	ele->Time = 0;
	ele->q_init[0] = 1.0f;
	ele->q_init[1] = 0.0f;
	ele->q_init[2] = 0.0f;
	ele->q_init[3] = 0.0f;
	ele->q_init_flag = false; 
	for(int i =1;i<=7;i++)
	{
		ele->P_k[(i-1)*7 + i-1] = 1.0f;
	}
	ele->circle_loop = 0;
}

bool Ahrs_Init(sAHRS *ele)
{
	Dprintf("\r\n%s Init...\r\n",ele->name);
	HAL_Delay(50);    //等地信号来临
	Adis_Calc(adis);
	Gps_Calc(gps);
	if(adis->Update==false || gps->MagUpdate==false)return false;
	arm_mat_init_f32(&ele->matA,3,1,adis->AccFil);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatAcc);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	arm_mat_init_f32(&ele->matA,3,1,adis->GyrFil);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatGyr);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	arm_mat_init_f32(&ele->matA,3,1,gps->MagFil);
	arm_mat_init_f32(&ele->matB,3,1,ele->DatMag);
	arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	//欧拉角计算
	float sinR,cosR,sinP,cosP,cosY,sinY;
	ele->Ang[0] = atan2f(ele->DatAcc[1],ele->DatAcc[2]);
	ele->Ang[1] = -asinf(ele->DatAcc[0]/adis->OneG);
	arm_sin_cos_f32(ele->Ang[0]*R2D,&sinR,&cosR);
	arm_sin_cos_f32(ele->Ang[1]*R2D,&sinP,&cosP);
	ele->Ang[2] = atan2f(-ele->DatMag[1]*cosR + ele->DatMag[2]*sinR,
		ele->DatMag[0]*cosP + ele->DatMag[1]*sinP*sinR + ele->DatMag[2]*sinP*cosR);
	//欧拉角转四元数
	arm_sin_cos_f32(ele->Ang[0]/2.0f*R2D,&sinR,&cosR);
	arm_sin_cos_f32(ele->Ang[1]/2.0f*R2D,&sinP,&cosP);
	arm_sin_cos_f32(ele->Ang[2]/2.0f*R2D,&sinY,&cosY);
	ele->w=cosY*cosP*cosR+sinY*sinP*sinR;
	ele->x=cosY*cosP*sinR-sinY*sinP*cosR;
	ele->y=cosY*sinP*cosR+sinY*cosP*sinR;
	ele->z=sinY*cosP*cosR-cosY*sinP*sinR;
	ele->Cb2n[0][0] = 1.0f-2.0f*(ele->y*ele->y + ele->z*ele->z);
	ele->Cb2n[0][1] =      2.0f*(ele->x*ele->y - ele->w*ele->z);
	ele->Cb2n[0][2] =      2.0f*(ele->x*ele->z + ele->w*ele->y);
	ele->Cb2n[1][0] =      2.0f*(ele->x*ele->y + ele->w*ele->z);
	ele->Cb2n[1][1] = 1.0f-2.0f*(ele->x*ele->x + ele->z*ele->z);
	ele->Cb2n[1][2] =      2.0f*(ele->y*ele->z - ele->w*ele->x);
	ele->Cb2n[2][0] =      2.0f*(ele->x*ele->z - ele->w*ele->y);
	ele->Cb2n[2][1] =      2.0f*(ele->y*ele->z + ele->w*ele->x);
	ele->Cb2n[2][2] = 1.0f-2.0f*(ele->x*ele->x + ele->y*ele->y);
	if(Ahrs_Calc(ele)==false)
	{
		Dprintf("%s Init.[NO]\r\n",ele->name);
		return false;
	}
	Dprintf("%s Init.[OK]\r\n",ele->name);
	return true;
}

bool Ahrs_Calc(sAHRS *ele)
{
	Tim_Calc(&ele->Tim);   //计时
	//传感器值获取
	if(adis->Update==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,adis->AccFil);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatAcc);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
		arm_mat_init_f32(&ele->matA,3,1,adis->GyrFil);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatGyr);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	}
	if(gps->MagUpdate==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,gps->MagFil);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatMag);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	}
	
	ele->DatAcc[0] = 1.0612f * (ele->DatAcc[0] - 0.3950f);
	ele->DatAcc[1] = 0.9994f * (ele->DatAcc[1] - 0.2206f);
	ele->DatAcc[2] = 1.0015f * (ele->DatAcc[2] + 0.2155f);
	/***************角速度获取*****************/
	ele->pqr[0] = ele->DatGyr[0];
	ele->pqr[1] = ele->DatGyr[1];
	ele->pqr[2] = ele->DatGyr[2];
	/****************角度获取******************/
	// 重力实测的重力向量
	float ax = ele->DatAcc[0], ay = ele->DatAcc[1], az = ele->DatAcc[2];
	float recipNorm_a = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm_a;
	ay *= recipNorm_a;
	az *= recipNorm_a;
	// 上一个四元数解算出来的重力向量，伪常量
	float vx = ele->Cb2n[2][0];
	float vy = ele->Cb2n[2][1];
	float vz = ele->Cb2n[2][2];
	// 规范化测量的磁场强度
	float mx = ele->DatMag[0], my = ele->DatMag[1], mz = ele->DatMag[2];
	float recipNorm_h = invSqrt(mx*mx + my*my + mz*mz);
	mx *= recipNorm_h;
	my *= recipNorm_h;
	mz *= recipNorm_h;
	// 计算参考地磁方向，伪常量
	float hx = mx*ele->Cb2n[0][0] + my*ele->Cb2n[0][1] + mz*ele->Cb2n[0][2];
	float hy = mx*ele->Cb2n[1][0] + my*ele->Cb2n[1][1] + mz*ele->Cb2n[1][2];
	float hz = mx*ele->Cb2n[2][0] + my*ele->Cb2n[2][1] + mz*ele->Cb2n[2][2];
	float bx = sqrtf((hx*hx) + (hy*hy));
	float bz = hz;
	// 计算常量在机体坐标下的表示。
	float rx = bx*ele->Cb2n[0][0] + bz*ele->Cb2n[2][0];
	float ry = bx*ele->Cb2n[0][1] + bz*ele->Cb2n[2][1];
	float rz = bx*ele->Cb2n[0][2] + bz*ele->Cb2n[2][2];	
	// pi参数控制
	const static float Kp = 0.7f;//4.2f  
	const static float Ki = 0.001f;//0.02f
//	float Kp = pidX->signal;
//	float Ki = pidY->signal;
	// 构造增量旋转。 
	float gx = ele->DatGyr[0], gy = ele->DatGyr[1], gz = ele->DatGyr[2]; //单位：rad/s
	float dx, dy, dz;
	float compass = 1.0f;
	float acc = 1.0f;
	if(gps->MagUpdate == false)compass = 0.0f;
	float ex = (ay*vz - az*vy)*acc;         //	float ex = (ay*vz - az*vy)*acc + (my*rz - mz*ry)*compass;
	float ey = (az*vx - ax*vz)*acc;         //	float ey = (az*vx - ax*vz)*acc + (mz*rx - mx*rz)*compass;
	float ez = (mx*ry - my*rx)*compass;     //	float ez = (ax*vy - ay*vx)*acc + (mx*ry - my*rx)*compass;
	ele->exInt += ex;
	ele->eyInt += ey;
	ele->ezInt += ez;
	dx = (gx + ex*Kp + ele->exInt*Ki)*ele->halfDt;
	dy = (gy + ey*Kp + ele->eyInt*Ki)*ele->halfDt;
	dz = (gz + ez*Kp + ele->ezInt*Ki)*ele->halfDt;
	// 融合，四元数乘法。[注][2016-1-18]实质上也是四元素微分方程的毕卡求解法之一阶近似算法
	float q0 = ele->w;
	float q1 = ele->x;
	float q2 = ele->y;
	float q3 = ele->z;
	ele->w = q0    - q1*dx - q2*dy - q3*dz;
	ele->x = q0*dx + q1    + q2*dz - q3*dy;
	ele->y = q0*dy - q1*dz + q2    + q3*dx;
	ele->z = q0*dz + q1*dy - q2*dx + q3;
	// 规范化四元数，防止发散。
	float recipNorm_q = invSqrt(ele->w*ele->w + ele->x*ele->x + ele->y*ele->y + ele->z*ele->z);
	ele->w *= recipNorm_q;
	ele->x *= recipNorm_q;
	ele->y *= recipNorm_q;
	ele->z *= recipNorm_q;
	//机体坐标系转换到北东地坐标系
	ele->Cb2n[0][0] = 1.0f-2.0f*(ele->y*ele->y + ele->z*ele->z);
	ele->Cb2n[0][1] =      2.0f*(ele->x*ele->y - ele->w*ele->z);
	ele->Cb2n[0][2] =      2.0f*(ele->x*ele->z + ele->w*ele->y);
	ele->Cb2n[1][0] =      2.0f*(ele->x*ele->y + ele->w*ele->z);
	ele->Cb2n[1][1] = 1.0f-2.0f*(ele->x*ele->x + ele->z*ele->z);
	ele->Cb2n[1][2] =      2.0f*(ele->y*ele->z - ele->w*ele->x);
	ele->Cb2n[2][0] =      2.0f*(ele->x*ele->z - ele->w*ele->y);
	ele->Cb2n[2][1] =      2.0f*(ele->y*ele->z + ele->w*ele->x);
	ele->Cb2n[2][2] = 1.0f-2.0f*(ele->x*ele->x + ele->y*ele->y);
	//四元数转欧拉角
	ele->Ang[0] =  atan2f(2.0f * ele->y * ele->z + 2.0f * ele->w * ele->x, -2.0f * ele->x * ele->x - 2.0f * ele->y * ele->y + 1.0f) - ele->AngOff[0];
	ele->Ang[1] =  asinf(fConstrain(-2.0f * ele->x * ele->z + 2.0f * ele->w * ele->y,-1.0f,1.0f)) - ele->AngOff[1];
	ele->Ang[2] =  atan2f(2.0f * ele->x * ele->y + 2.0f * ele->w * ele->z, -2.0f * ele->y * ele->y - 2.0f * ele->z * ele->z + 1.0f) - ele->AngOff[2];
  //重力夹角
	float sinY,cosY;
	arm_sin_cos_f32(ahrs->Ang[2]*R2D,&sinY,&cosY);
	float Dt = 2.0f*ele->halfDt;
	ele->Fac = fConstrain(1.0f/ele->Cb2n[2][2] - 1.0f,0.0f,0.7f); //相当于1/(cos(r)cos(p))-1
	/*************NED加速度计算**************/
	ele->AccB[0]= ele->DatAcc[0];ele->AccB[1] = ele->DatAcc[1];ele->AccB[2] = ele->DatAcc[2];//by czh
	ele->Acc[0] = ele->AccB[0]*ele->Cb2n[0][0] + ele->AccB[1]*ele->Cb2n[0][1] + ele->AccB[2]*ele->Cb2n[0][2];
	ele->Acc[1] = ele->AccB[0]*ele->Cb2n[1][0] + ele->AccB[1]*ele->Cb2n[1][1] + ele->AccB[2]*ele->Cb2n[1][2];
	ele->Acc[2] = ele->AccB[0]*ele->Cb2n[2][0] + ele->AccB[1]*ele->Cb2n[2][1] + ele->AccB[2]*ele->Cb2n[2][2]+adis->OneG;
	/*************NED速度计算**************/
	float Kv[3] = {0.8f,0.8f,0.8f};
	if(gps->Update == false){Kv[0]=0.0f;Kv[1]=0.0f;}
	ele->Uvw[0] = (1.0f-Kv[0])*(ele->Uvw[0] + ele->Acc[0]*Dt) + Kv[0]*gps->NED_spd[0];
	ele->Uvw[1] = (1.0f-Kv[1])*(ele->Uvw[1] + ele->Acc[1]*Dt) + Kv[1]*gps->NED_spd[1];
	float TmpW = 0.0f, FACv=0.0f;
	if(ms5611[0].Err == ERR_NONE && ms5611[0].Sta == STA_RUN){TmpW+=-ms5611[0].AltSlope;FACv += 1.0f;} 
	if(ms5611[1].Err == ERR_NONE && ms5611[1].Sta == STA_RUN){TmpW+=-ms5611[1].AltSlope;FACv += 1.0f;} 
	if(exitx->Update[1] == true && -ele->Xyz[2]<3.0f) {TmpW+=-exitx->DatSlope[1];FACv += 1.0f;}  
	if(FACv>0.0f) TmpW /= FACv;
	else Kv[2]=0.0f;
	ele->Uvw[2] = (1.0f-Kv[2])*(ele->Uvw[2] + ele->Acc[2]*Dt) + Kv[2]*TmpW;
	/*************NED位置计算**************/
	float Ks[3] = {0.8f,0.8f,0.8f};
	if(gps->Update == false){Ks[0]=0.0f;Ks[1]=0.0f;}
	ele->Xyz[0] = (1.0f-Ks[0])*(ele->Xyz[0] + ele->Uvw[0]*Dt) + Ks[0]*gps->NED[0];
	ele->Xyz[1] = (1.0f-Ks[1])*(ele->Xyz[1] + ele->Uvw[1]*Dt) + Ks[1]*gps->NED[1];
	float TmpZ = 0.0f, FACs=0.0f;
	if(ms5611[0].Err == ERR_NONE && ms5611[0].Sta == STA_RUN){TmpZ+=-ms5611[0].AltSlope;FACs += 1.0f;} 
	if(ms5611[1].Err == ERR_NONE && ms5611[1].Sta == STA_RUN){TmpZ+=-ms5611[1].AltSlope;FACs += 1.0f;} 
	if(FACs>0.0f) TmpZ /= FACs;
	else Ks[2]=0.0f;
	FACs = (-ele->Xyz[2]-3.0f)*0.5f;
	FACs = fConstrain(FACs,0.0f,1.0f);
	ele->Xyz[2] = (1.0f-Ks[2])*(ele->Xyz[2] + ele->Uvw[2]*Dt) + Ks[2]*(FACs*TmpZ + (1.0f-FACs)*-exitx->DatFil[1]*ele->Cb2n[2][2]);
  /*************机头坐标系计算**************/
	ele->AccH[0] = ele->Acc[0]*cosY + ele->Acc[1]*sinY;
	ele->AccH[1] = -ele->Acc[0]*sinY + ele->Acc[1]*cosY;
	ele->AccH[2] = ele->Acc[2];
	ele->UvwH[0] = ele->Uvw[0]*cosY + ele->Uvw[1]*sinY;
	ele->UvwH[1] = -ele->Uvw[0]*sinY + ele->Uvw[1]*cosY;
	ele->UvwH[2] = ele->Uvw[2];
	ele->XyzH[0] = ele->Xyz[0]*cosY + ele->Xyz[1]*sinY;
	ele->XyzH[1] = -ele->Xyz[0]*sinY + ele->Xyz[1]*cosY;
	ele->XyzH[2] = ele->Xyz[2];
	/****************计算结束****************/
	ele->Update = true;
	Tim_Calc(&ele->Tim);   //计时
	ele->Time = ele->Tim.OUT;
	return ele->Update;
}

/***************EKF 姿态部分 2018.5.3 CHR***************/
//详情请查看matlab文件
void EKF_Attitude(const float x_p[7], const float acc_m[3], const float mag_m[3],
                  const float wgm[3], const float P[49], float dt, float x_k[7],
                  float x_n[7], float PP[49])
{
  float scale;
  float c;
  float y;
  float euler_angle_idx_1;
  float mag_calib_idx_0;
  float mag_calib_idx_1;
  float Obs_value_p[4];
  float H[28];
  float absxk;
  float t;
  float b_c;
  float c_c;
  float d_c;
  float b_H[28];
  float W_X[16];
  signed char ipiv[4];
  int k;
  int kBcol;
  int jp;
	//R阵
  static const float fv0[16] = { 0.0049F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0049F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0049F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0081F };

  int j;
  int e_c;
  int ix;
  int i;
  int jAcol;
  float K[28];
  float b_acc_m[4];
  float c_acc_m[4];
  float b_K[7];
		
  scale = atan2f(acc_m[1], acc_m[2]);
  c = -acc_m[0] * invSqrt(acc_m[0] * acc_m[0] + acc_m[1] * acc_m[1] +acc_m[2] * acc_m[2]);
		
  if (c < -1.0F) {
    y = -1.0F;
  } else if (c > 1.0F) {
    y = 1.0F;
  } else {
    y = c;
  }
  euler_angle_idx_1 = (float)asin(y);

  mag_calib_idx_0 = (mag_m[0] * (float)cos(euler_angle_idx_1) + mag_m[1] *
                     (float)sin((float)asin(y)) * (float)sin(atan2f
    (acc_m[1], acc_m[2]))) + mag_m[2] * (float)sin(euler_angle_idx_1) * (float)
    cos(scale);
  mag_calib_idx_1 = -mag_m[1] * (float)cos(scale) + mag_m[2] * (float)sin(scale);
  Obs_value_p[0] = 2.0F * (x_p[1] * x_p[3] - x_p[0] * x_p[2]);
  Obs_value_p[1] = 2.0F * (x_p[2] * x_p[3] + x_p[0] * x_p[1]);
  Obs_value_p[2] = ((x_p[0] * x_p[0] - x_p[1] * x_p[1]) - x_p[2] * x_p[2]) +
    x_p[3] * x_p[3];
  Obs_value_p[3] = atan2f(2.0F * (x_p[1] * x_p[2] + x_p[0] * x_p[3]),
    ((x_p[0] * x_p[0] + x_p[1] * x_p[1]) - x_p[2] * x_p[2]) - x_p[3] * x_p[3]);
  memset(&H[0], 0, 28U * sizeof(float));
  c = 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2]);
  scale = x_p[2] * x_p[1] + x_p[0] * x_p[3];
  absxk = 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2]);
  t = x_p[2] * x_p[1] + x_p[0] * x_p[3];
  y = 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2]);
  b_c = x_p[2] * x_p[1] + x_p[0] * x_p[3];
  c_c = 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2]);
  d_c = x_p[2] * x_p[1] + x_p[0] * x_p[3];
  H[0] = -2.0F * x_p[2];
  H[4] = 2.0F * x_p[3];
  H[8] = -2.0F * x_p[0];
  H[12] = 2.0F * x_p[1];
  H[1] = 2.0F * x_p[1];
  H[5] = 2.0F * x_p[0];
  H[9] = 2.0F * x_p[3];
  H[13] = 2.0F * x_p[2];
  H[2] = 2.0F * x_p[0];
  H[6] = -2.0F * x_p[1];
  H[10] = -2.0F * x_p[2];
  H[14] = 2.0F * x_p[3];
  H[3] = 2.0F * x_p[3] * (1.0F - 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2])) /
    ((1.0F - c) * (1.0F - c) + 4.0F * (scale * scale));
  H[7] = 2.0F * x_p[2] * (1.0F - 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2])) /
    ((1.0F - absxk) * (1.0F - absxk) + 4.0F * (t * t));
  H[11] = (2.0F * x_p[1] * (1.0F - 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2])) +
           4.0F * x_p[2] * (x_p[1] * x_p[2] + x_p[0] * x_p[3])) / ((1.0F - y) *
    (1.0F - y) + 4.0F * (b_c * b_c));
  H[15] = (2.0F * x_p[0] * (1.0F - 2.0F * (x_p[3] * x_p[3] + x_p[2] * x_p[2])) +
           4.0F * x_p[3] * (x_p[0] * x_p[3] + x_p[1] * x_p[2])) / ((1.0F - c_c) *
    (1.0F - c_c) + 4.0F * (d_c * d_c));
  for (k = 0; k < 4; k++) {
    for (kBcol = 0; kBcol < 7; kBcol++) {
      b_H[k + (kBcol << 2)] = 0.0F;
      for (jp = 0; jp < 7; jp++) {
        b_H[k + (kBcol << 2)] += H[k + (jp << 2)] * P[jp + 7 * kBcol];
      }
    }

    for (kBcol = 0; kBcol < 4; kBcol++) {
      scale = 0.0F;
      for (jp = 0; jp < 7; jp++) {
        scale += b_H[k + (jp << 2)] * H[kBcol + (jp << 2)];
      }

      W_X[k + (kBcol << 2)] = scale + fv0[k + (kBcol << 2)];
    }

    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 3; j++) {
    e_c = j * 5;
    kBcol = 0;
    ix = e_c;
    scale = (float)fabs(W_X[e_c]);
    for (k = 2; k <= 4 - j; k++) {
      ix++;
      absxk = (float)fabs(W_X[ix]);
      if (absxk > scale) {
        kBcol = k - 1;
        scale = absxk;
      }
    }

    if (W_X[e_c + kBcol] != 0.0F) {
      if (kBcol != 0) {
        ipiv[j] = (signed char)((j + kBcol) + 1);
        ix = j;
        kBcol += j;
        for (k = 0; k < 4; k++) {
          scale = W_X[ix];
          W_X[ix] = W_X[kBcol];
          W_X[kBcol] = scale;
          ix += 4;
          kBcol += 4;
        }
      }

      k = (e_c - j) + 4;
      for (i = e_c + 1; i + 1 <= k; i++) {
        W_X[i] /= W_X[e_c];
      }
    }

    jp = e_c;
    jAcol = e_c + 4;
    for (kBcol = 1; kBcol <= 3 - j; kBcol++) {
      scale = W_X[jAcol];
      if (W_X[jAcol] != 0.0F) {
        ix = e_c + 1;
        k = (jp - j) + 8;
        for (i = 5 + jp; i + 1 <= k; i++) {
          W_X[i] += W_X[ix] * -scale;
          ix++;
        }
      }

      jAcol += 4;
      jp += 4;
    }
  }

  for (k = 0; k < 7; k++) {
    for (kBcol = 0; kBcol < 4; kBcol++) {
      K[k + 7 * kBcol] = 0.0F;
      for (jp = 0; jp < 7; jp++) {
        K[k + 7 * kBcol] += P[k + 7 * jp] * H[kBcol + (jp << 2)];
      }
    }
  }

  for (j = 0; j < 4; j++) {
    jp = 7 * j;
    jAcol = j << 2;
    for (k = 1; k <= j; k++) {
      kBcol = 7 * (k - 1);
      if (W_X[(k + jAcol) - 1] != 0.0F) {
        for (i = 0; i < 7; i++) {
          K[i + jp] -= W_X[(k + jAcol) - 1] * K[i + kBcol];
        }
      }
    }

    scale = 1.0F / W_X[j + jAcol];
    for (i = 0; i < 7; i++) {
      K[i + jp] *= scale;
    }
  }

  for (j = 3; j >= 0; j += -1) {
    jp = 7 * j;
    jAcol = (j << 2) - 1;
    for (k = j + 2; k < 5; k++) {
      kBcol = 7 * (k - 1);
      if (W_X[k + jAcol] != 0.0F) {
        for (i = 0; i < 7; i++) {
          K[i + jp] -= W_X[k + jAcol] * K[i + kBcol];
        }
      }
    }
  }

  for (kBcol = 2; kBcol >= 0; kBcol += -1) {
    if (ipiv[kBcol] != kBcol + 1) {
      jp = ipiv[kBcol] - 1;
      for (jAcol = 0; jAcol < 7; jAcol++) {
        scale = K[jAcol + 7 * kBcol];
        K[jAcol + 7 * kBcol] = K[jAcol + 7 * jp];
        K[jAcol + 7 * jp] = scale;
      }
    }
  }

  for (k = 0; k < 3; k++) {
    b_acc_m[k] = acc_m[k] ;
  }

  if (mag_calib_idx_0 != 0.0F) {
    b_acc_m[3] = atan2f(mag_calib_idx_1, mag_calib_idx_0);
  } else if (mag_calib_idx_1 < 0.0F) {
    b_acc_m[3] = -1.57079637F;
  } else {
    b_acc_m[3] = 1.57079637F;
  }

  for (k = 0; k < 4; k++) {
    c_acc_m[k] = b_acc_m[k] - Obs_value_p[k];
  }
	if(c_acc_m[3]>PI) c_acc_m[3]-=2*PI;
	else if (c_acc_m[3]<-PI) c_acc_m[3]+=2*PI;

  for (k = 0; k < 7; k++) {
    b_K[k] = 0.0F;
    for (kBcol = 0; kBcol < 4; kBcol++) {
      b_K[k] += K[k + 7 * kBcol] * c_acc_m[kBcol];
    }

    x_k[k] = x_p[k] + b_K[k];
  }

  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 4; k++) {
    absxk = (float)fabs(x_k[k]);
    if (absxk > scale) {
      t = scale / absxk;
      y = 1.0F + y * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  y = scale * (float)sqrt(y);
  for (k = 0; k < 4; k++) {
    x_k[k] /= y;
  }

  memset(&F[0], 0, 49U * sizeof(float));
  W_X[0] = 0.0F;
  W_X[4] = -(wgm[0] - x_k[4]);
  W_X[8] = -(wgm[1] - x_k[5]);
  W_X[12] = -(wgm[2] - x_k[6]);
  W_X[1] = wgm[0] - x_k[4];
  W_X[5] = 0.0F;
  W_X[9] = wgm[2] - x_k[6];
  W_X[13] = -(wgm[1] - x_k[5]);
  W_X[2] = wgm[1] - x_k[5];
  W_X[6] = -(wgm[2] - x_k[6]);
  W_X[10] = 0.0F;
  W_X[14] = wgm[0] - x_k[4];
  W_X[3] = wgm[2] - x_k[6];
  W_X[7] = wgm[1] - x_k[5];
  W_X[11] = -(wgm[0] - x_k[4]);
  W_X[15] = 0.0F;
  for (k = 0; k < 4; k++) {
    for (kBcol = 0; kBcol < 4; kBcol++) {
      F[kBcol + 7 * k] = 0.5F * W_X[kBcol + (k << 2)];
    }
  }

  F[28] = x_k[1];
  F[35] = x_k[2];
  F[42] = x_k[3];
  F[29] = -x_k[0];
  F[36] = x_k[3];
  F[43] = -x_k[2];
  F[30] = -x_k[3];
  F[37] = -x_k[0];
  F[44] = x_k[1];
  F[31] = x_k[2];
  F[38] = -x_k[1];
  F[45] = -x_k[0];
  for (k = 0; k < 49; k++) {
    tao[k] = (float)a[k] * dt;
    F[k] = (float)a[k] + F[k] * dt;
  }

  for (k = 0; k < 7; k++) {
    for (kBcol = 0; kBcol < 7; kBcol++) {
      scale = 0.0F;
      for (jp = 0; jp < 4; jp++) {
        scale += K[k + 7 * jp] * H[jp + (kBcol << 2)];
      }

      b_a[k + 7 * kBcol] = (float)a[k + 7 * kBcol] - scale;
    }

    for (kBcol = 0; kBcol < 7; kBcol++) {
      c_a[k + 7 * kBcol] = 0.0F;
      for (jp = 0; jp < 7; jp++) {
        c_a[k + 7 * kBcol] += b_a[k + 7 * jp] * P[jp + 7 * kBcol];
      }
    }
  }
  for (k = 0; k < 5; k++) {
    for (kBcol = 0; kBcol < 5; kBcol++) {
      b_a[k + 7 * kBcol] = 0.0F;
      b_tao[k + 7 * kBcol] = 0.0f;
      for (jp = 0; jp < 7; jp++) {
        b_a[k + 7 * kBcol] += F[k + 7 * jp] * c_a[jp + 7 * kBcol];
        b_tao[k + 7 * kBcol] += tao[k + 7 * jp] * b[jp + 7 * kBcol];
      }
    }

    for (kBcol = 0; kBcol < 7; kBcol++) {
      b_F[k + 7 * kBcol] = 0.0F;
      c_tao[k + 7 * kBcol] = 0.0F;
      for (jp = 0; jp < 7; jp++) {
        b_F[k + 7 * kBcol] += b_a[k + 7 * jp] * F[kBcol + 7 * jp];
        c_tao[k + 7 * kBcol] += b_tao[k + 7 * jp] * tao[kBcol + 7 * jp];
      }
    }
  }

  for (k = 0; k < 7; k++) {
    for (kBcol = 0; kBcol < 7; kBcol++) {
      PP[kBcol + 7 * k] = b_F[kBcol + 7 * k] + c_tao[kBcol + 7 * k];
    }
  }

  for (k = 0; k < 7; k++) {
    for (kBcol = 0; kBcol < 7; kBcol++) {
      b_a[kBcol + 7 * k] = (PP[kBcol + 7 * k] + PP[k + 7 * kBcol]) / 2.0F;
    }
  }

  for (k = 0; k < 7; k++) {
    for (kBcol = 0; kBcol < 7; kBcol++) {
      PP[kBcol + 7 * k] = b_a[kBcol + 7 * k];
    }
  }

  c = 0.5F * dt;
  for (k = 0; k < 4; k++) {
    b_acc_m[k] = 0.0F;
    for (kBcol = 0; kBcol < 4; kBcol++) {
      b_acc_m[k] += c * W_X[k + (kBcol << 2)] * x_k[kBcol];
    }

    x_n[k] = x_k[k] + b_acc_m[k];
  }

  for (k = 0; k < 3; k++) {
    x_n[4 + k] = x_k[4 + k];
  }
}


void euler2quaternion(float euler[3],float q[4])
{
	float cphi = cos(0.5f * euler[0]);
  float sphi = sin(0.5f * euler[0]);
	float cthe = cos(0.5f * euler[1]);
	float sthe = sin(0.5f * euler[1]);
	float cpsi = cos(0.5f * euler[2]);
	float spsi = sin(0.5f * euler[2]);
	q[0] = cphi * cthe * cpsi + sphi * sthe * spsi;
	q[1] = sphi * cthe * cpsi - cphi * sthe * spsi;
	q[2] = cphi * sthe * cpsi + sphi * cthe * spsi;
	q[3] = cphi * cthe * spsi - sphi * sthe * cpsi;
}

bool Ahrs_Calc_EKF(sAHRS *ele)
{
	Tim_Calc(&ele->Tim);   //计时
	//传感器值获取
	if(adis->Update==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,adis->AccFil_3nd);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatAcc);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
		arm_mat_init_f32(&ele->matA,3,1,adis->GyrFil_2nd);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatGyr);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	}
	if(gps->MagUpdate==true)
	{
		arm_mat_init_f32(&ele->matA,3,1,gps->MagFil);
		arm_mat_init_f32(&ele->matB,3,1,ele->DatMag);
		arm_mat_mult_f32(&ele->mat33,&ele->matA,&ele->matB);
	}
	ele->ahrs_update_time = ele->Tim.CNT;
	ele->gps_ins_update_flag = true;
	/**************加速度计校正****************/
//	ele->DatAcc[0] = 1.007f * (ele->DatAcc[0] + 0.0438f);
//	ele->DatAcc[1] = 1.005f * (ele->DatAcc[1] + 0.0289f);
//	ele->DatAcc[2] = 1.008f * (ele->DatAcc[2] + 0.2560f);
	/***************角速度获取*****************/
	ele->pqr[0] = ele->DatGyr[0];
	ele->pqr[1] = ele->DatGyr[1];
	ele->pqr[2] = ele->DatGyr[2];
	
	float w_rad[3];
	w_rad[0] = ele->pqr[0];
	w_rad[1] = ele->pqr[1];
	w_rad[2] = ele->pqr[2];
	/****************角度获取******************/
	// 重力实测的重力向量
	float ax = ele->DatAcc[0], ay = ele->DatAcc[1], az = ele->DatAcc[2];
	float recipNorm_a = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm_a;
	ay *= recipNorm_a;
	az *= recipNorm_a;
	float a_norm[3];
	a_norm[0] = ax;
	a_norm[1] = ay;
	a_norm[2] = az;
	// 规范化测量的磁场强度
	float mx = ele->DatMag[0], my = ele->DatMag[1], mz = ele->DatMag[2];
	float recipNorm_h = invSqrt(mx*mx + my*my + mz*mz);
	mx *= recipNorm_h;
	my *= recipNorm_h;
	mz *= recipNorm_h;
	float m_norm[3];
	m_norm[0] = mx;
	m_norm[1] = my;
	m_norm[2] = mz;
	
	ele->circle_loop ++;
	if(ele->circle_loop == 200)
	{
		//EKF 初始化步骤
		float mag_calib[2];
		ele->Ang_Init[0] = atan2f(a_norm[1],a_norm[2]);
		ele->Ang_Init[1] = asin(-a_norm[0]);
		mag_calib[0] =  m_norm[0] * cos(ele->Ang_Init[1]) + m_norm[1] * sin(ele->Ang_Init[1]) * sin(ele->Ang_Init[0]) + m_norm[2] * sin(ele->Ang_Init[1]) * cos(ele->Ang_Init[0]);
		mag_calib[1] = -m_norm[1] * cos(ele->Ang_Init[0]) + m_norm[2] * sin(ele->Ang_Init[0]);
		if (mag_calib[0] != 0)
		{
			ele->Ang_Init[2] = atan2f(mag_calib[1], mag_calib[0]);
		}
		else
		{
			if (mag_calib[1] < 0)
				ele->Ang_Init[2] = -PI/2;
			else
				ele->Ang_Init[2] = PI/2;
		}
		euler2quaternion(ele->Ang_Init,ele->q_init);
		ele->q_init_flag = true;
		//EKF初始化值
		ele->x_p[0] = ele->q_init[0];
		ele->x_p[1] = ele->q_init[1];
		ele->x_p[2] = ele->q_init[2];
		ele->x_p[3] = ele->q_init[3];
		ele->x_p[4] = 0.0f;
		ele->x_p[5] = 0.0f;
		ele->x_p[6] = 0.0f;
}
	else if(ele->circle_loop>64530)
	{
		ele->circle_loop = 250;
	}
	//EKF迭代部分
	if(ele->q_init_flag)
	{
		float x_k[7]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
		float x_n[7]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
		float PP[49];
		for(int i=0;i<49;i++) PP[i] = 0.0f;
		EKF_Attitude(ele->x_p,a_norm,m_norm,w_rad,ele->P_k,2*ele->halfDt,x_k,x_n,PP);
		for (int i=0;i<7;i++)
		{
			ele->x_p[i] = x_n[i];
		}
		for (int i=0;i<49;i++)
		{
			ele->P_k[i] = PP[i];
		}
		//EKF得到的四元数信息
		ele->w = x_k[0];
		ele->x = x_k[1];
		ele->y = x_k[2];
		ele->z = x_k[3];
		
		ele->w1 = ele->x_p[0];
		ele->x1 = ele->x_p[1];
		ele->y1 = ele->x_p[2];
		ele->z1 = ele->x_p[3];
		
		//坐标系转换
		ele->Cb2n[0][0] = 1.0f-2.0f*(ele->y*ele->y + ele->z*ele->z);
		ele->Cb2n[0][1] =      2.0f*(ele->x*ele->y - ele->w*ele->z);
		ele->Cb2n[0][2] =      2.0f*(ele->x*ele->z + ele->w*ele->y);
		ele->Cb2n[1][0] =      2.0f*(ele->x*ele->y + ele->w*ele->z);
		ele->Cb2n[1][1] = 1.0f-2.0f*(ele->x*ele->x + ele->z*ele->z);
		ele->Cb2n[1][2] =      2.0f*(ele->y*ele->z - ele->w*ele->x);
		ele->Cb2n[2][0] =      2.0f*(ele->x*ele->z - ele->w*ele->y);
		ele->Cb2n[2][1] =      2.0f*(ele->y*ele->z + ele->w*ele->x);
		ele->Cb2n[2][2] = 1.0f-2.0f*(ele->x*ele->x + ele->y*ele->y);
		//四元数转欧拉角
		ele->Ang[0] =  atan2f(2.0f * ele->y * ele->z + 2.0f * ele->w * ele->x, -2.0f * ele->x * ele->x - 2.0f * ele->y * ele->y + 1.0f) - ele->AngOff[0];
		ele->Ang[1] =  asinf(fConstrain(-2.0f * ele->x * ele->z + 2.0f * ele->w * ele->y,-1.0f,1.0f)) - ele->AngOff[1];
		ele->Ang[2] =  atan2f(2.0f * ele->x * ele->y + 2.0f * ele->w * ele->z, -2.0f * ele->y * ele->y - 2.0f * ele->z * ele->z + 1.0f) - ele->AngOff[2];

		ele->Ang1[0] =  atan2f(2.0f * ele->y1 * ele->z1 + 2.0f * ele->w1 * ele->x1, -2.0f * ele->x1 * ele->x1 - 2.0f * ele->y1 * ele->y1 + 1.0f) - ele->AngOff[0];
		ele->Ang1[1] =  asinf(fConstrain(-2.0f * ele->x1 * ele->z1 + 2.0f * ele->w1 * ele->y1,-1.0f,1.0f)) - ele->AngOff[1];
		ele->Ang1[2] =  atan2f(2.0f * ele->x1 * ele->y1 + 2.0f * ele->w1 * ele->z1, -2.0f * ele->y1 * ele->y1 - 2.0f * ele->z1 * ele->z1 + 1.0f) - ele->AngOff[2];


//		//重力夹角
//		float sinY,cosY;
//		arm_sin_cos_f32(ahrs->Ang[2]*R2D,&sinY,&cosY);
//		float Dt = 2.0f*ele->halfDt;
//		ele->Fac = fConstrain(1.0f/ele->Cb2n[2][2] - 1.0f,0.0f,0.7f); //相当于1/(cos(r)cos(p))-1
		
		//机体坐标系下加速度
		ele->AccB[0]= -ele->DatAcc[0];ele->AccB[1]=-ele->DatAcc[1];ele->AccB[2]=-ele->DatAcc[2];
		ele->Acc[0] = ele->AccB[0]*ele->Cb2n[0][0] + ele->AccB[1]*ele->Cb2n[0][1] + ele->AccB[2]*ele->Cb2n[0][2];
		ele->Acc[1] = ele->AccB[0]*ele->Cb2n[1][0] + ele->AccB[1]*ele->Cb2n[1][1] + ele->AccB[2]*ele->Cb2n[1][2];
		ele->Acc[2] = ele->AccB[0]*ele->Cb2n[2][0] + ele->AccB[1]*ele->Cb2n[2][1] + ele->AccB[2]*ele->Cb2n[2][2]+adis->OneG;
//		/*************NED速度计算**************/
//		float Kv[3] = {0.8f,0.8f,0.8f};
//		if(gps->Update == false){Kv[0]=0.0f;Kv[1]=0.0f;}
//		ele->Uvw[0] = (1.0f-Kv[0])*(ele->Uvw[0] + ele->Acc[0]*Dt) + Kv[0]*gps->NED_spd[0];
//		ele->Uvw[1] = (1.0f-Kv[1])*(ele->Uvw[1] + ele->Acc[1]*Dt) + Kv[1]*gps->NED_spd[1];
//		float TmpW = 0.0f, FACv=0.0f;
//		if(ms5611[0].Err == ERR_NONE && ms5611[0].Sta == STA_RUN){TmpW+=-ms5611[0].AltSlope;FACv += 1.0f;} 
//		if(ms5611[1].Err == ERR_NONE && ms5611[1].Sta == STA_RUN){TmpW+=-ms5611[1].AltSlope;FACv += 1.0f;} 
//		if(exitx->Update[1] == true && -ele->Xyz[2]<3.0f) {TmpW+=-exitx->DatSlope[1];FACv += 1.0f;}  
//		if(FACv>0.0f) TmpW /= FACv;
//		else Kv[2]=0.0f;
//		ele->Uvw[2] = (1.0f-Kv[2])*(ele->Uvw[2] + ele->Acc[2]*Dt) + Kv[2]*TmpW;
//		/*************NED位置计算**************/
//		float Ks[3] = {0.8f,0.8f,0.8f};
//		if(gps->Update == false){Ks[0]=0.0f;Ks[1]=0.0f;}
//		ele->Xyz[0] = (1.0f-Ks[0])*(ele->Xyz[0] + ele->Uvw[0]*Dt) + Ks[0]*gps->NED[0];
//		ele->Xyz[1] = (1.0f-Ks[1])*(ele->Xyz[1] + ele->Uvw[1]*Dt) + Ks[1]*gps->NED[1];
//		float TmpZ = 0.0f, FACs=0.0f;
//		if(ms5611[0].Err == ERR_NONE && ms5611[0].Sta == STA_RUN){TmpZ+=-ms5611[0].AltSlope;FACs += 1.0f;} 
//		if(ms5611[1].Err == ERR_NONE && ms5611[1].Sta == STA_RUN){TmpZ+=-ms5611[1].AltSlope;FACs += 1.0f;} 
//		if(FACs>0.0f) TmpZ /= FACs;
//		else Ks[2]=0.0f;
//		FACs = (-ele->Xyz[2]-3.0f)*0.5f;
//		FACs = fConstrain(FACs,0.0f,1.0f);
//		ele->Xyz[2] = (1.0f-Ks[2])*(ele->Xyz[2] + ele->Uvw[2]*Dt) + Ks[2]*(FACs*TmpZ + (1.0f-FACs)*-exitx->DatFil[1]*ele->Cb2n[2][2]);
//		/*************机头坐标系计算**************/
//		ele->AccH[0] = ele->Acc[0]*cosY + ele->Acc[1]*sinY;
//		ele->AccH[1] = -ele->Acc[0]*sinY + ele->Acc[1]*cosY;
//		ele->AccH[2] = ele->Acc[2];
//		ele->UvwH[0] = ele->Uvw[0]*cosY + ele->Uvw[1]*sinY;
//		ele->UvwH[1] = -ele->Uvw[0]*sinY + ele->Uvw[1]*cosY;
//		ele->UvwH[2] = ele->Uvw[2];
//		ele->XyzH[0] = ele->Xyz[0]*cosY + ele->Xyz[1]*sinY;
//		ele->XyzH[1] = -ele->Xyz[0]*sinY + ele->Xyz[1]*cosY;
//		ele->XyzH[2] = ele->Xyz[2];
		/****************计算结束****************/
		ele->Update = true;
		Tim_Calc(&ele->Tim);   //计时
		ele->Time = ele->Tim.OUT;
	}
	return ele->Update;
}

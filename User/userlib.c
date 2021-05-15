#include "userlib.h"

/***************************************************\
功能：
  延时函数，打印函数，常用函数
说明：
  1、编译器加入预编译ARM_MATH_CM7,__FPU_PRESENT=1
  2、CMSIS里添加DSP库。
  3、根据硬件配置修改h文件标注需要修改的部分。 
\***************************************************/

/******************驱动程序****************/
void DMA_CLEAR_FLAG_ALL(DMA_HandleTypeDef *dmax)
{		
	u32 ele=(u32)dmax->Instance;
  (ele == (u32)DMA1_Stream0)? (DMA1->LIFCR=0x0000003D) :\
	(ele == (u32)DMA1_Stream1)? (DMA1->LIFCR=0x00000F40) :\
	(ele == (u32)DMA1_Stream2)? (DMA1->LIFCR=0x003D0000) :\
	(ele == (u32)DMA1_Stream3)? (DMA1->LIFCR=0x0F400000) :\
	(ele == (u32)DMA1_Stream4)? (DMA1->HIFCR=0x0000003D) :\
	(ele == (u32)DMA1_Stream5)? (DMA1->HIFCR=0x00000F40) :\
	(ele == (u32)DMA1_Stream6)? (DMA1->HIFCR=0x003D0000) :\
	(ele == (u32)DMA1_Stream7)? (DMA1->HIFCR=0x0F400000) :\
	(ele == (u32)DMA2_Stream0)? (DMA2->LIFCR=0x0000003D) :\
	(ele == (u32)DMA2_Stream1)? (DMA2->LIFCR=0x00000F40) :\
	(ele == (u32)DMA2_Stream2)? (DMA2->LIFCR=0x003D0000) :\
	(ele == (u32)DMA2_Stream3)? (DMA2->LIFCR=0x0F400000) :\
	(ele == (u32)DMA2_Stream4)? (DMA2->HIFCR=0x0000003D) :\
	(ele == (u32)DMA2_Stream5)? (DMA2->HIFCR=0x00000F40) :\
	(ele == (u32)DMA2_Stream6)? (DMA2->HIFCR=0x003D0000) :\
	(DMA2->HIFCR=0x0F400000);
}

/******************功能函数****************/
//系统软复位
void Sys_Rst(void)
{   
	SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 

static bool TIM_FIRST_USE=true;
//计时，调用两次的时间差放在OUT里边，CNT存放上一次数据。
void Tim_Calc(sTIM *timx)
{
	if(TIM_FIRST_USE){HAL_TIM_Base_Start(&TIM_COUNT);TIM_FIRST_USE=false;}    //计时器
	timx->OUT=TIM_COUNT.Instance->CNT - timx->CNT;
	timx->CNT=TIM_COUNT.Instance->CNT;
}

//用户延时函数，1us计时，最大延时为4294s
void User_Delay(u32 nus)
{
	if(TIM_FIRST_USE){HAL_TIM_Base_Start(&TIM_COUNT);TIM_FIRST_USE=false;}    //计时器
	u32 StrTim = TIM_COUNT.Instance->CNT;
	while(TIM_COUNT.Instance->CNT-StrTim<=nus);
}

//调试串口打印函数
char cache[UART_CACHE_NUM];
void Dprintf(const char * _format,...)
{
	#if UART_DEBUG_EN==1
	va_list ap;
	va_start(ap,_format);
	vsnprintf(cache,UART_CACHE_NUM,_format,ap);
	va_end(ap);
	HAL_UART_Transmit(&UART_DEBUG, (u8 *)cache, strlen(cache), UART_TIMEOUT);
	#endif
}

//发送串口打印函数
void Sprintf(const char * _format,...)
{
	va_list ap;
	va_start(ap,_format);
	vsnprintf(cache,UART_CACHE_NUM,_format,ap);
	va_end(ap);
	HAL_UART_Transmit(&UART_SEND, (u8 *)cache, strlen(cache), UART_TIMEOUT);
}

//字符串转矩阵，将"+X+Y+Z"转成 +1 0 +1 1 +1 2
bool Dir_Trans(s8 Dir[6],const char DirChr[6])
{
	for(int i=0;i<6;)
	{
		switch(DirChr[i])
		{
			case '+':Dir[i]= 1;break;
			case '-':Dir[i]=-1;break;
			default:return false;
		}
		i++;
		switch(DirChr[i])
		{
			case 'X':Dir[i]=0;break;
			case 'Y':Dir[i]=1;break;
			case 'Z':Dir[i]=2;break;
			default:return false;
		}
		i++;
	}
	return true;
}

//判断符号位
float Sign(float value)
{
	if(value>=0.0f)return 1.0f;
	else return -1.0f;
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

float Norm3(float *data)
{
	return sqrtf(SQR(data[0])+SQR(data[1])+SQR(data[2]));
}

//限幅函数
float fConstrain(float Input, float minValue, float maxValue)
{
	if (Input < minValue) return minValue;
	else if (Input > maxValue) return maxValue;
	else return Input;
}

//限幅函数
s16 iConstrain(s16 Input, s16 minValue, s16 maxValue)
{
	if (Input < minValue) return minValue;
	else if (Input > maxValue) return maxValue;
	else return Input;
}

//循环限幅函数
float LoopConstrain(float Input, float minValue, float maxValue)
{
	if(Input>=maxValue)return LoopConstrain(Input-(maxValue-minValue),minValue,maxValue);
	if(Input<minValue)return LoopConstrain(Input+(maxValue-minValue),minValue,maxValue);
	return Input;
}

//弧度格式化为-PI~PI
float Rad_Format(float Ang)
{
	return LoopConstrain(Ang,-PI,PI);
}

//角度格式化为-180~180
float Theta_Format(float Ang)
{
	return LoopConstrain(Ang,-180.0f,180.0f);
}

//滑窗均值滤波器，CMD:0重新初始化，1正常滤波，2使用同个FIL滤波。
void SlideFilt(float *Dat,float *DatRel,u8 num,sCNT *Filt,u8 Cmd)
{
	switch(Cmd)
	{
		case 0:Filt->CNT=1;break;
		case 1:if(Filt->CNT < Filt->CCR)Filt->CNT++;break;
		case 2:break;
	}
	if(Filt->CNT==1)
	{
		for(int i=0;i<num;i++)
		{
			Dat[i]=DatRel[i];
		}
	}
	for(int i=0;i<num;i++)
	{
		Dat[i]=(Dat[i]*(Filt->CNT-1)+DatRel[i])/Filt->CNT;
	}
}

//字符分割
u8 StrSeg(const char *str,char chr,char *para[],u8 num)
{
	u8 i;
	char *pStr = (char *)strchr(str,chr);       
	if(pStr==NULL)return 0;         //找不到','
	for(i=0;i<num;i++)
	{
		pStr = strchr(pStr,chr);
		if(pStr==NULL)break;      
		para[i]=++pStr;
	}
	return i;  
}

//数据伪更新滤除器，same:当前数据是否相同，返回值：true：正常更新 false：伪更新
bool DataCheck(bool same,sCNT *Check)
{
	if(same==false)
	{
		Check->CNT=0;
		return true;
	}
	if(++Check->CNT>=Check->CCR)return false;
	return true;
}

void LineFit(float* Y,u16 N,float *ab) //y=ax+b
{
	float SumXX=0.0f,SumX=0.0f,SumXY=0.0f,SumY=0.0f;
	for(u16 i=0;i<N;i++)
	{
		SumXX += i*i;
		SumX  += i;
		SumXY += Y[i]*i;
		SumY  += Y[i];
	}
	ab[0] = (SumXY*N-SumX*SumY)/(SumXX*N-SumX*SumX);
	ab[1] = (SumXX*SumY-SumX*SumXY)/(SumXX*N-SumX*SumX);
}

//圆形拟合，d：源数据 N:个数 圆圆心与半径
void circleFit(float d[][2],u16 N,float circleOrigin[],float *circleRadius)
{
	int i;
	float x2,y2;
	float sum_x = 0.0f, sum_y = 0.0f;
	float sum_x2 = 0.0f, sum_y2 = 0.0f;
	float sum_x3 = 0.0f, sum_y3 = 0.0f;
	float sum_xy = 0.0f, sum_xy2 = 0.0f, sum_x2y = 0.0f;
	float C, D, E, G, H;
	float a, b, c;
	circleOrigin[0] = 0.0f;
	circleOrigin[1] = 0.0f;
	*circleRadius = 0.0f;
	for(i=0;i<N;i++)
	{
		x2 = d[i][0]*d[i][0];
		y2 = d[i][1]*d[i][1];
		sum_x += d[i][0];
		sum_y += d[i][1];
		sum_x2 += x2;
		sum_y2 += y2;
		sum_x3 += x2*d[i][0];
		sum_y3 += y2*d[i][1];
		sum_xy += d[i][0]*d[i][1];
		sum_xy2 += d[i][0]*y2;
		sum_x2y += d[i][1]*x2;
	}
	C = N * sum_x2 - sum_x * sum_x;
	D = N * sum_xy - sum_x * sum_y;
	E = N * sum_x3 + N * sum_xy2 - (sum_x2 + sum_y2) * sum_x;
	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
	circleOrigin[0] = a / (-2);
	circleOrigin[1] = b / (-2);
	*circleRadius = sqrt(a * a + b * b - 4 * c) / 2;
}

//球形拟合  d：源数据 N:个数 球圆心与半径
void sphereFit(float d[][3],u16 N,u16 MaxIterations,float Err,u16 Population[][3],float SphereOrigin[],float *SphereRadius)
{
  u8  c;
	u16 i, Iterations;
	float    s[3], s2[3], s3[3], sum[3], sum2[3], sum3[3];
	float    x2sum[3], y2sum[3], z2sum[3];
	float    xy_sum, xz_sum, yz_sum;
	float    XY, XZ, YZ, X2Z, Y2X, Y2Z, Z2X, X2Y, Z2Y;
	float    QS, QB, Q0, Q1, Q2;
	float    R2, C[3], C2[3], Delta[3], Denom[3];
	float    F0, F1, F2, F3, F4;
	float    di2[3];
	float    SizeR;
	for (c = 0; c <= 2; c++)
	{
		s[c] = s2[c] = s3[c] = sum[c] = x2sum[c] = y2sum[c] = z2sum[c] = 0.0f;
		Population[0][c] = Population[1][c] = 0;
	}
	xy_sum = xz_sum = yz_sum = 0.0f;
	for (i = 0; i < N; i++)
	{
		for (c = 0; c <= 2; c++)
		{
				di2[c] = SQR(d[i][c]);
				s[c]  += d[i][c];
				s2[c] += di2[c];
				s3[c] += di2[c] * d[i][c];
				Population[d[i][c] > 0.0f][c]++;
		}
		xy_sum += d[i][0] * d[i][1];
		xz_sum += d[i][0] * d[i][2];
		yz_sum += d[i][1] * d[i][2];
		x2sum[1] += di2[0] * d[i][1];
		x2sum[2] += di2[0] * d[i][2];
		y2sum[0] += di2[1] * d[i][0];
		y2sum[2] += di2[1] * d[i][2];
		z2sum[0] += di2[2] * d[i][0];
		z2sum[1] += di2[2] * d[i][1];
	}
	SizeR = 1.0f / (float) N;
	for (c = 0; c <= 2; c++)
	{
		sum[c]  = s[c]  * SizeR; //sum( X[n]   )
		sum2[c] = s2[c] * SizeR; //sum( X[n]^2 )
		sum3[c] = s3[c] * SizeR; //sum( X[n]^3 )
	}
	XY = xy_sum * SizeR;         //sum( X[n] * Y[n] )
	XZ = xz_sum * SizeR;         //sum( X[n] * Z[n] )
	YZ = yz_sum * SizeR;         //sum( Y[n] * Z[n] )
	X2Y = x2sum[1] * SizeR;  //sum( X[n]^2 * Y[n] )
	X2Z = x2sum[2] * SizeR;  //sum( X[n]^2 * Z[n] )
	Y2X = y2sum[0] * SizeR;  //sum( Y[n]^2 * X[n] )
	Y2Z = y2sum[2] * SizeR;  //sum( Y[n]^2 * Z[n] )
	Z2X = z2sum[0] * SizeR;  //sum( Z[n]^2 * X[n] )
	Z2Y = z2sum[1] * SizeR;  //sum( Z[n]^2 * Y[n] )
	//Reduction of multiplications
	F0 = sum2[0] + sum2[1] + sum2[2];
	F1 = 0.5f * F0;
	F2 = -8.0f * (sum3[0] + Y2X + Z2X);
	F3 = -8.0f * (X2Y + sum3[1] + Z2Y);
	F4 = -8.0f * (X2Z + Y2Z + sum3[2]);
	for (c = 0; c <= 2; c++)
	{
		C[c]  = sum[c];
		C2[c] = SQR(C[c]);
	}
	QS = C2[0] + C2[1] + C2[2];
	QB = -2.0f * (SQR(C[0]) + SQR(C[1]) + SQR(C[2]));
	R2 = F0 + QB + QS;
	Q0 = 0.5f * (QS - R2);
	Q1 = F1 + Q0;
	Q2 = 8.0f * (QS - R2 + QB + F0);
	Iterations = 0;
	do
	{
		for (c = 0; c <= 2; c++)
		{
			Denom[c] = Q2 + 16.0f * (C2[c] - 2.0f * C[c] * sum[c] + sum2[c]);
			if (Denom[c] == 0.0f)
				Denom[c] = 1.0f;
		}
		Delta[0] = -((F2 + 16.0f * (C[1] * XY + C[2] * XZ + sum[0] * (-C2[0] - Q0)
																		+ C[0] * (sum2[0] + Q1 - C[2] * sum[2] - C[1] * sum[1]))) / Denom[0]);
		Delta[1] = -((F3 + 16.0f * (C[0] * XY + C[2] * YZ + sum[1] * (-C2[1] - Q0)
																		+ C[1] * (sum2[1] + Q1 - C[0] * sum[0] - C[2] * sum[2]))) / Denom[1]);
		Delta[2] = -((F4 + 16.0f * (C[0] * XZ + C[1] * YZ + sum[2] * (-C2[2] - Q0)
																		+ C[2] * (sum2[2] + Q1 - C[0] * sum[0] - C[1] * sum[1]))) / Denom[2]);
		for (c = 0; c <= 2; c++)
		{
			C[c] += Delta[c];
			C2[c] = SQR(C[c]);
		}
		QS = C2[0] + C2[1] + C2[2];
		QB = -2.0f * (C[0] * sum[0] + C[1] * sum[1] + C[2] * sum[2]);
		R2 = F0 + QB + QS;
		Q0 = 0.5f * (QS - R2);
		Q1 = F1 + Q0;
		Q2 = 8.0f * (QS - R2 + QB + F0);
		Iterations++;
	}
	while ((Iterations < 50) || ((Iterations < MaxIterations) && ((SQR(Delta[0]) + SQR(Delta[1]) + SQR(Delta[2])) > Err)));
	for (c = 0; c <= 2; c++)  SphereOrigin[c] = C[c];
	*SphereRadius = sqrt(R2);
}


static float _flt_inv_fact[] =
{
  1.0f / 1.0f,                    // 1/0!
  1.0f / 1.0f,                    // 1/1!
  1.0f / 2.0f,                    // 1/2!
  1.0f / 6.0f,                    // 1/3!
  1.0f / 24.0f,                   // 1/4!
  1.0f / 120.0f,                  // 1/5!
  1.0f / 720.0f,                  // 1/6!
  1.0f / 5040.0f,                 // 1/7!
  1.0f / 40320.0f,                // 1/8!
  1.0f/ 362880.0f,               // 1/9!
  1.0f / 3628800.0f,              // 1/10!
};

#define M_E 2.71828
#define M_E2    (M_E * M_E)
#define M_E4    (M_E2 * M_E2)
#define M_E8    (M_E4 * M_E4)
#define M_E16   (M_E8 * M_E8)
#define M_E32   (M_E16 * M_E16)
#define M_E64   (M_E32 * M_E32)
#define M_E128  (M_E64 * M_E64)
#define M_E256  (M_E128 * M_E128)
#define M_E512  (M_E256 * M_E256)
#define M_E1024 (M_E512 * M_E512)

/************************************************************************
 * Private Data
 ************************************************************************/

static double _expi_square_tbl[10] =
{
  M_E,                          // e^1
  M_E2,                         // e^2
  M_E4,                         // e^4
  M_E8,                         // e^8
  M_E16,                        // e^16
  M_E32,                        // e^32
  M_E64,                        // e^64
  M_E128,                       // e^128
  M_E256,                       // e^256
  M_E512,                       // e^512
};

float sq(float val)
{
    return powf(val, 2);
}

float powf(float b, float e)
{
  return expf(e * logf(b));
}

double lib_expi(size_t n)
{
  size_t i;
  double val;

  if (n > 1024)
    {
      return INFINITY;
    }

  val = 1.0f;

  for (i = 0; n; i++)
    {
      if (n & (1 << i))
        {
          n   &= ~(1 << i);
          val *= _expi_square_tbl[i];
        }
    }

  return val;
}

float expf(float x)
{
  size_t int_part;
  bool invert;
  float value;
  float x0;
  size_t i;

  if (x == 0)
    {
      return 1;
    }
  else if (x < 0)
    {
      invert = true;
      x = -x;
    }
  else
    {
      invert = false;
    }

  /* Extract integer component */

  int_part = (size_t) x;

  /* set x to fractional component */

  x -= (float)int_part;

  /* Perform Taylor series approximation with eleven terms */

  value = 0.0f;
  x0 = 1.0f;
  for (i = 0; i < 10; i++)
    {
      value += x0 * _flt_inv_fact[i];
      x0 *= x;
    }

  /* Multiply by exp of the integer component */

  value *= lib_expi(int_part);

  if (invert)
    {
      return (1.0f / value);
    }
  else
    {
      return value;
    }
}




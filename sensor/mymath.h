#ifndef __MYMATH_H__
#define __MYMATH_H__

#include "stm32f10x.h"

#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define TAN_MAP_SIZE    256
#define PI 3.14159265358979323846
#define DEG_TO_RAD 0.017453292519943259
#define RAD_TO_DEG 57.3f
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0) ? (safe_value) : ((numerator)/(denominator)) )
#define ABS(x) ( (x)>0?(x):-(x) )
#define VAL_LIMIT( x,min,max ) x=( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )
#define _MIN(a, b) ((a) < (b) ? (a) : (b))
#define _MAX(a, b) ((a) > (b) ? (a) : (b))
#define my_atan(x) fast_atan2(x,1)

float my_abs(float f);
float fast_atan2(float y, float x);
float my_sqrt(float number);
float Q_rsqrt(float number);
float my_asin(float x);
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deathzoom(float x,float ,float zoom);
float my_deathzoom_2(float x,float ,float zoom);

float my_deadzone_p(float x,float zone);
float my_deadzone_n(float x,float zone);

float To_180_degrees(float x);
double To_180_degrees_db(double x);
//float my_pow_2_curve(float in,float a,float max);
//float safe_div(float numerator ,float denominator,float sv);
float linear_interpolation_5(float range[5],float interpolation[5],float in);//range 必须从小到大
double my_pow(double num, double m);

#endif

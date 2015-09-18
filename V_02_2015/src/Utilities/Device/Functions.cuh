/*
 * Functions.h
 *
 *  Created on: Jun 5, 2014
 *      Author: nicolin
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_


////////////////////////////////////////////////////////////////////////////////
// constructors
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float2 make_float2(float s)
{
    return make_float2(s, s);
}
inline  __device__ float2 make_float2(float3 a)
{
    return make_float2(a.x, a.y);
}
inline  __device__ float2 make_float2(int2 a)
{
    return make_float2(float(a.x), float(a.y));
}
inline  __device__ float2 make_float2(uint2 a)
{
    return make_float2(float(a.x), float(a.y));
}

inline  __device__ int2 make_int2(int s)
{
    return make_int2(s, s);
}
inline  __device__ int2 make_int2(int3 a)
{
    return make_int2(a.x, a.y);
}
inline  __device__ int2 make_int2(uint2 a)
{
    return make_int2(int(a.x), int(a.y));
}
inline  __device__ int2 make_int2(float2 a)
{
    return make_int2(int(a.x), int(a.y));
}

inline  __device__ uint2 make_uint2(uint s)
{
    return make_uint2(s, s);
}
inline  __device__ uint2 make_uint2(uint3 a)
{
    return make_uint2(a.x, a.y);
}
inline  __device__ uint2 make_uint2(int2 a)
{
    return make_uint2(uint(a.x), uint(a.y));
}

inline  __device__ float3 make_float3(float s)
{
    return make_float3(s, s, s);
}
inline  __device__ float3 make_float3(float2 a)
{
    return make_float3(a.x, a.y, 0.0f);
}
inline  __device__ float3 make_float3(float2 a, float s)
{
    return make_float3(a.x, a.y, s);
}
inline  __device__ float3 make_float3(float4 a)
{
    return make_float3(a.x, a.y, a.z);
}
inline  __device__ float3 make_float3(int3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}
inline  __device__ float3 make_float3(uint3 a)
{
    return make_float3(float(a.x), float(a.y), float(a.z));
}

inline  __device__ int3 make_int3(int s)
{
    return make_int3(s, s, s);
}
inline  __device__ int3 make_int3(int2 a)
{
    return make_int3(a.x, a.y, 0);
}
inline  __device__ int3 make_int3(int2 a, int s)
{
    return make_int3(a.x, a.y, s);
}
inline  __device__ int3 make_int3(uint3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}
inline  __device__ int3 make_int3(float3 a)
{
    return make_int3(int(a.x), int(a.y), int(a.z));
}

inline  __device__ uint3 make_uint3(uint s)
{
    return make_uint3(s, s, s);
}
inline  __device__ uint3 make_uint3(uint2 a)
{
    return make_uint3(a.x, a.y, 0);
}
inline  __device__ uint3 make_uint3(uint2 a, uint s)
{
    return make_uint3(a.x, a.y, s);
}
inline  __device__ uint3 make_uint3(uint4 a)
{
    return make_uint3(a.x, a.y, a.z);
}
inline  __device__ uint3 make_uint3(int3 a)
{
    return make_uint3(uint(a.x), uint(a.y), uint(a.z));
}

inline  __device__ float4 make_float4(float s)
{
    return make_float4(s, s, s, s);
}
inline  __device__ float4 make_float4(float3 a)
{
    return make_float4(a.x, a.y, a.z, 0.0f);
}
inline  __device__ float4 make_float4(float3 a, float w)
{
    return make_float4(a.x, a.y, a.z, w);
}
inline  __device__ float4 make_float4(int4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}
inline  __device__ float4 make_float4(uint4 a)
{
    return make_float4(float(a.x), float(a.y), float(a.z), float(a.w));
}

inline  __device__ int4 make_int4(int s)
{
    return make_int4(s, s, s, s);
}
inline  __device__ int4 make_int4(int3 a)
{
    return make_int4(a.x, a.y, a.z, 0);
}
inline  __device__ int4 make_int4(int3 a, int w)
{
    return make_int4(a.x, a.y, a.z, w);
}
inline  __device__ int4 make_int4(uint4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}
inline  __device__ int4 make_int4(float4 a)
{
    return make_int4(int(a.x), int(a.y), int(a.z), int(a.w));
}


inline  __device__ uint4 make_uint4(uint s)
{
    return make_uint4(s, s, s, s);
}
inline  __device__ uint4 make_uint4(uint3 a)
{
    return make_uint4(a.x, a.y, a.z, 0);
}
inline  __device__ uint4 make_uint4(uint3 a, uint w)
{
    return make_uint4(a.x, a.y, a.z, w);
}
inline  __device__ uint4 make_uint4(int4 a)
{
    return make_uint4(uint(a.x), uint(a.y), uint(a.z), uint(a.w));
}

////////////////////////////////////////////////////////////////////////////////
// dot product
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float dot(float2 a, float2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline  __device__ float dot(float3 a, float3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline  __device__ float dot(float4 a, float4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline  __device__ int dot(int2 a, int2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline  __device__ int dot(int3 a, int3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline  __device__ int dot(int4 a, int4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

inline  __device__ uint dot(uint2 a, uint2 b)
{
    return a.x * b.x + a.y * b.y;
}
inline  __device__ uint dot(uint3 a, uint3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline  __device__ uint dot(uint4 a, uint4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

////////////////////////////////////////////////////////////////////////////////
// length
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float length(float2 v)
{
    return sqrtf(dot(v, v));
}
inline  __device__ float length(float3 v)
{
    return sqrtf(dot(v, v));
}
inline  __device__ float length(float4 v)
{
    return sqrtf(dot(v, v));
}

////////////////////////////////////////////////////////////////////////////////
// normalize
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float2 normalize(float2 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}
inline  __device__ float3 normalize(float3 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}
inline  __device__ float4 normalize(float4 v)
{
    float invLen = rsqrtf(dot(v, v));
    return v * invLen;
}

////////////////////////////////////////////////////////////////////////////////
// floor
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float2 floorf(float2 v)
{
    return make_float2(floorf(v.x), floorf(v.y));
}
inline  __device__ float3 floorf(float3 v)
{
    return make_float3(floorf(v.x), floorf(v.y), floorf(v.z));
}
inline  __device__ float4 floorf(float4 v)
{
    return make_float4(floorf(v.x), floorf(v.y), floorf(v.z), floorf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// frac - returns the fractional portion of a scalar or each vector component
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float fracf(float v)
{
    return v - floorf(v);
}
inline  __device__ float2 fracf(float2 v)
{
    return make_float2(fracf(v.x), fracf(v.y));
}
inline  __device__ float3 fracf(float3 v)
{
    return make_float3(fracf(v.x), fracf(v.y), fracf(v.z));
}
inline  __device__ float4 fracf(float4 v)
{
    return make_float4(fracf(v.x), fracf(v.y), fracf(v.z), fracf(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// fmod
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float2 fmodf(float2 a, float2 b)
{
    return make_float2(fmodf(a.x, b.x), fmodf(a.y, b.y));
}
inline  __device__ float3 fmodf(float3 a, float3 b)
{
    return make_float3(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z));
}
inline  __device__ float4 fmodf(float4 a, float4 b)
{
    return make_float4(fmodf(a.x, b.x), fmodf(a.y, b.y), fmodf(a.z, b.z), fmodf(a.w, b.w));
}

////////////////////////////////////////////////////////////////////////////////
// absolute value
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float2 fabs(float2 v)
{
    return make_float2(fabs(v.x), fabs(v.y));
}
inline  __device__ float3 fabs(float3 v)
{
    return make_float3(fabs(v.x), fabs(v.y), fabs(v.z));
}
inline  __device__ float4 fabs(float4 v)
{
    return make_float4(fabs(v.x), fabs(v.y), fabs(v.z), fabs(v.w));
}

inline  __device__ int2 abs(int2 v)
{
    return make_int2(abs(v.x), abs(v.y));
}
inline  __device__ int3 abs(int3 v)
{
    return make_int3(abs(v.x), abs(v.y), abs(v.z));
}
inline  __device__ int4 abs(int4 v)
{
    return make_int4(abs(v.x), abs(v.y), abs(v.z), abs(v.w));
}

////////////////////////////////////////////////////////////////////////////////
// reflect
// - returns reflection of incident ray I around surface normal N
// - N should be normalized, reflected vector's length is equal to length of I
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float3 reflect(float3 i, float3 n)
{
    return i - 2.0f * n * dot(n,i);
}

////////////////////////////////////////////////////////////////////////////////
// cross product
////////////////////////////////////////////////////////////////////////////////

inline  __device__ float3 cross(float3 a, float3 b)
{
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}


/* Quartations */
/*-----------------------------------------------------------------------------*/
__device__  inline float lengthD(Quaterion Q1)
{

    float sqrd = Q1.w*Q1.w +Q1.x*Q1.x + Q1.y*Q1.y + Q1.z*Q1.z;

    float n;

    if(sqrd>0.0f)
    {
    n = (sqrt( sqrd ));
    }
    else
    {
    	n=0.0f;
    }
    return n;
}
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
__device__  inline Quaterion normaliseD(Quaterion Q1)
{
	Quaterion q;
    float sqrd = Q1.w*Q1.w +Q1.x*Q1.x + Q1.y*Q1.y + Q1.z*Q1.z;

    float n;

    if(sqrd>0.0f)
    {
    n = 1.0f/(sqrt( sqrd ));
    }
    else
    {
    	n=0.0f;
    }
    q.w  = Q1.w*n;
    q.x  = Q1.x*n;
    q.y  = Q1.y*n;
    q.z  = Q1.z*n;

    return q;
}
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*            Returns a unit quaterion representing the rotation               */
                       /* Theta in Degrees */
/*-----------------------------------------------------------------------------*/
__device__ inline Quaterion d_make_quaterion_DEG(float theta,float3 Axis)
{
  Quaterion q;

  theta = ((2.0f*3.14152f)/360.0f)*theta;
  q.w= cos(theta/2.0f);
  q.x= sin(theta/2.0f)*Axis.x;
  q.y= sin(theta/2.0f)*Axis.y;
  q.z= sin(theta/2.0f)*Axis.z;

  return normaliseD(q);
}
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__device__  inline Quaterion d_make_quaterion_DEG(float theta,float x,float y,float z)
{
  Quaterion q;
  theta = ((2.0f*3.14152f)/360.0f)*theta;
  q.w= cos(theta/2.0f);
  q.x= sin(theta/2.0f)*x;
  q.y= sin(theta/2.0f)*y;
  q.z= sin(theta/2.0f)*z;

  return normaliseD(q);
}
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
/*            Returns a unit quaterion representing the rotation               */
                       /* Theta in Degrees */
/*-----------------------------------------------------------------------------*/
__device__ inline Quaterion d_make_quaterion_RAD(float theta,float3 Axis)
{
  Quaterion q;

  q.w= cos(theta/2.0f);
  q.x= sin(theta/2.0f)*Axis.x;
  q.y= sin(theta/2.0f)*Axis.y;
  q.z= sin(theta/2.0f)*Axis.z;

  return normaliseD(q);
}
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__device__  inline Quaterion d_make_quaterion_RAD(float theta,float x,float y,float z)
{
  Quaterion q;

  q.w= cos(theta/2.0f);
  q.x= sin(theta/2.0f)*x;
  q.y= sin(theta/2.0f)*y;
  q.z= sin(theta/2.0f)*z;

  return normaliseD(q);
}
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__device__  inline Quaterion make_quaterionD(float3 Vector)
{
  Quaterion q;

  q.w= 0.0f;
  q.x= Vector.x;
  q.y= Vector.y;
  q.z= Vector.z;

  return q;
}
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__device__  inline Quaterion make_IquaterionD()
{
  Quaterion q;

  q.w= 1.000000f;
  q.x= 0.000000f;
  q.y= 0.000000f;
  q.z= 0.000000f;

  return q;
}
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__device__  inline float3 make_vectorD(Quaterion Vector)
{
  float3 q;


  q.x= Vector.x;
  q.y= Vector.y;
  q.z= Vector.z;

  return q;
}
/*-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------*/
__device__  inline Quaterion conjugateD(Quaterion Q1)
{
  Quaterion q;
  q.w =  Q1.w;
  q.x = -Q1.x;
  q.y = -Q1.y;
  q.z = -Q1.z;

  return q;
 }
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__device__  Quaterion quart_axisAngleD(Quaterion Q1)
{
	Quaterion q;
	float angle = (2.000f*acos(Q1.w));
	float coeff =0.0f;
	if(angle>0.0f)
	{
	coeff = 1.0f/sin(angle*0.50000f);

	}

	q.w = angle*(180.0f/(2.0f*asin(1.0)));
	q.x = Q1.x*coeff;
	q.y = Q1.y*coeff;
	q.z = Q1.z*coeff;


	return q;
}
/*-----------------------------------------------------------------------------*/


#endif /* FUNCTIONS_H_ */

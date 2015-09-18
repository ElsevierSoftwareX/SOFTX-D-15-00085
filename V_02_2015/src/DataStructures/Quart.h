/*
 * Quart.h
 *
 *  Created on: Nov 13, 2012
 *      Author: nicolin
 */

#include "../Utilities/Common/Types.h"

#ifndef QUART_H_
#define QUART_H_



float length(Quaterion Q1)
{
	Quaterion q;
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


inline Quaterion normalise(Quaterion Q1)
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

/* Returns a unit quaterion representing the rotation*/
inline Quaterion make_quaterion(float theta,float3 Axis)
{
  Quaterion q;
  theta = ((2.0f*3.14152f)/360.0f)*theta;
  q.w= cos(theta/2.0f);
  q.x= sin(theta/2.0f)*Axis.x;
  q.y= sin(theta/2.0f)*Axis.y;
  q.z= sin(theta/2.0f)*Axis.z;

  return normalise(q);
}

 inline Quaterion make_quaterion(float theta,float x,float y,float z)
{
  Quaterion q;
  theta = ((2.0f*3.14152f)/360.0f)*theta;
  q.w= cos(theta/2.0f);
  q.x= sin(theta/2.0f)*x;
  q.y= sin(theta/2.0f)*y;
  q.z= sin(theta/2.0f)*z;

  return normalise(q);
}

inline Quaterion make_quaterion(float3 Vector)
{
  Quaterion q;

  q.w= 0.0f;
  q.x= Vector.x;
  q.y= Vector.y;
  q.z= Vector.z;

  return q;
}

inline float3 make_vector(Quaterion Vector)
{
  float3 q;


  q.x= Vector.x;
  q.y= Vector.y;
  q.z= Vector.z;

  return q;
}

inline Quaterion operator+(Quaterion Q1,Quaterion Q2)
{
   Quaterion q;
   q.w = Q1.w + Q2.w;
   q.x = Q1.x + Q2.x;
   q.y = Q1.y + Q2.y;
   q.z = Q1.z + Q2.z;

   return q;
}




 inline Quaterion operator*(Quaterion q1,Quaterion q2)
{
	Quaterion q;
    q.x =  (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y) + (q1.w * q2.x);
    q.y = -(q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x) + (q1.w * q2.y);
    q.z =  (q1.x * q2.y) - (q1.y * q2.x) + (q1.z * q2.w) + (q1.w * q2.z);
    q.w = -(q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z) + (q1.w * q2.w);
    return q;
}



inline Quaterion conjugate(Quaterion Q1)
{
  Quaterion q;
  q.w =  Q1.w;
  q.x = -Q1.x;
  q.y = -Q1.y;
  q.z = -Q1.z;

  return q;
 }




 Quaterion quart_axisAngle(Quaterion Q1)
{
	Quaterion q;
	float angle = (2.0f*acos(Q1.w));
	float coeff = 1.0f/sin(angle/2.0f);
	q.w = angle*57.30f;
	q.x = Q1.x*coeff;
	q.y = Q1.y*coeff;
	q.z = Q1.z*coeff;


	return q;
}

/*-----------------------------------------------------------*/
void Compute_Mul_Mat_Mat(float *A, float *B, float *C)
{
 // float C[9];

  for (unsigned int i = 0; i < 3; i++)
  {
     for (unsigned int j = 0; j < 3; j++)
     {
    	float sum = 0;

        for (unsigned int k = 0; k < 3; k++)
        {
           sum += (A[ i*3 + k ] * B[ k*3 + j ]);
        }

         C[i*3 + j] = sum;
      }
  }

  //return C;
}


#endif /* QUART_H_ */

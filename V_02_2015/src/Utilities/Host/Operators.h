/*
 * Operators.h
 *
 *  Created on: Jun 5, 2014
 *      Author: nicolin
 */

#ifndef OPERATORS_H_
#define OPERATORS_H_
#include "Functions.h"

/*---------------------------------------------------------------------------*/
 inline float3 operator+(float3 a, float3 b )
 {
   float3 c;

   c.x = a.x +b.x;
   c.y = a.y +b.y;
   c.z = a.z +b.z;

   return c;
 }

 /*---------------------------------------------------------------------------*/
 inline float3 operator-(float3 a, float3 b )
 {
   float3 c;
   c.x = a.x - b.x;
   c.y = a.y - b.y;
   c.z = a.z - b.z;
   return c;
 }

 /*---------------------------------------------------------------------------*/
 inline float3 operator/(float3 a, float3 b)
 {
     return make_float3(a.x / b.x, a.y / b.y, a.z / b.z);
 }

 /*---------------------------------------------------------------------------*/
 inline void operator+=(float3 &a, float b)
 {
     a.x += b;
     a.y += b;
     a.z += b;
 }

 /*---------------------------------------------------------------------------*/
 inline float3 operator*(float3 a, float3 b)
 {
     return make_float3(a.x * b.x, a.y * b.y, a.z * b.z);
 }
/*---------------------------------------------------------------------------*/

 inline float3 operator*(float b, float3 a)
 {
     return make_float3(b * a.x, b * a.y, b * a.z);
 }
 /*---------------------------------------------------------------------------*/
 inline float3 operator*(float3 a, float b)
 {
     return make_float3(a.x * b, a.y * b, a.z * b);
 }
 /*---------------------------------------------------------------------------*/
 inline float3 operator/(float3 a, float b)
 {
     return make_float3(a.x / b, a.y / b, a.z / b);
 }
 /*---------------------------------------------------------------------------*/
 inline  void operator/=(float3 &a, float b)
 {
     a.x /= b;
     a.y /= b;
     a.z /= b;
 }

 inline   void operator+=(float3 &a, float3 b)
 {
     a.x += b.x;
     a.y += b.y;
     a.z += b.z;
 }

 /*---------------------------------------------------------------------------*/
 inline  float3 operator/(float b, float3 a)
 {
     return make_float3(b / a.x, b / a.y, b / a.z);
 }
 /*---------------------------------------------------------------------------*/

#endif /* OPERATORS_H_ */

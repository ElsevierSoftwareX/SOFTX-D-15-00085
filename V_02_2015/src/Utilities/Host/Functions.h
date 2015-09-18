/*
 * Functions.h
 *
 *  Created on: Jun 5, 2014
 *      Author: nicolin
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <math.h>

/*---------------------------------------------------------------------------*/
 static inline float3 make_float3(float x, float y, float z)
 {
   float3 t; t.x = x; t.y = y; t.z = z; return t;
 }


 inline float dot(float3 a, float3 b)
 {
     return a.x * b.x + a.y * b.y + a.z * b.z;
 }
 /*---------------------------------------------------------------------------*/
 inline float length(float3 v)
 {
     return sqrtf(dot(v, v));
 }
 /*---------------------------------------------------------------------------*/
inline float3 zeroF()
 {
   float3 c;

   c.x = 0.0f;
   c.y = 0.0f;
   c.z = 0.0f;

   return c;
 }
/*---------------------------------------------------------------------------*/
inline float3 cross(float3 a, float3 b)
{
    return make_float3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
/*---------------------------------------------------------------------------*/

#endif /* FUNCTIONS_H_ */

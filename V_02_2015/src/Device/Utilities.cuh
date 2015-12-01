/* Author: Nicolin Govender   */
/* govender.nicolin@gmail.com */
/* Created: 12-January-2013   */




/*-----------------------------------------------------------------------------*/
__device__ void PrintQuartND(Quaterion Vec)
{
  printf("  %+10.5f %+10.5f %+10.5f  %+10.5f   \n", Vec.w, Vec.x,Vec.y,Vec.z);
}
/*-----------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*                 Checks if the point is on the surface                   */
/*---------------------------------------------------------------------------*/
__device__ bool is_PointOnSurface( int num_vertex, float3 *surface_vertex,
		                           int *v_order,float3 point, float SArea,
		                           float TOL, float3 Offset = make_float3(0.0f)     )
{

	float3 V1,V2;
	float area = 0.000000f;

	/* Loop over all vertex
	 * Select pairs and compute the vector from the test point*/
	for( int i=0; i< (num_vertex-1); i++)
	{

	  V1 = point - surface_vertex[ v_order[i]   ];
	  V2 = point - surface_vertex[ v_order[i+1] ];

	  /* Using the cross product to get the are between the two vector*/
	  area = area + 0.500000f*length(cross(V1,V2));

	}
    /* Loop misses the area between first and last vertex*/
	V1 = point - surface_vertex[ v_order[ num_vertex-1 ] ];
	V2 = point - surface_vertex[ v_order[ 0            ] ];

	area = area + 0.500000f*length( cross(V1,V2) );

	if ( (fabs(area-SArea)/SArea)*100.0000f <= TOL)
	{
		return true;
	}

	return false;

}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
/*   (8) Perform a Detailed check using the Separating plane method          */
/*       Returns: Point on A | Normal Away from A | Penetration Distance     */
/*---------------------------------------------------------------------------*/
__device__ bool isPointinPoly( int     num_faces_A,
                               float3  Pos_A,
                               Plane   *faceListA,
                               float3  point)
{
   bool is_collision_vp;
   float  vertex_dis ;

/*---------------------------------------------------------------------------*/
/*                     TYPE 1: PLANE SEARCH FACE A                           */
/*---------------------------------------------------------------------------*/

      /* Loop over Planes of Particle A */
	  for ( uint i=0; i<num_faces_A; i++)
	  {

           is_collision_vp = false; /* check for intersection */

           /* _|_ distance between vertex and plane */
           vertex_dis = dot( faceListA[i].normal,( point - faceListA[i].centroid ) );

           /* if we past the plane report collision */
	       if( vertex_dis <= 0.000100f )
	       {
	          is_collision_vp = true; /* point inside*/

	       }

           if( !is_collision_vp )
           {
              return false;
           }
	   } /* End of checking all planes of A */

	  return true;
}



/*---------------------------------------------------------------------------*/
     /* (6) Quick Check for Vertex-Face SP O( F1V2 + F2V1 )= O(2FV)*/
/*---------------------------------------------------------------------------*/
__device__ bool is_SP_Poly_Poly( uint num_faces_A, Plane *Faces_A,
		                         float3 Pos_B, uint num_vertex_B, float3 *Vertex_B,
		                         int *face_sp=0)
{
   bool   is_contact;
   float  vertex_dis ;

/*---------------------------------------------------------------------------*/
/*                     TYPE 1: PLANE SEARCH FACE A                           */
/*---------------------------------------------------------------------------*/

      /* Loop over Planes of Particle A */
	  for ( uint i=0; i<num_faces_A; i++)
	  {
         is_contact = false; /* check for intersection */

         if( dot( Faces_A[i].normal, ( Pos_B - Faces_A[i].centroid ) )>0.0 )
         {
           /* Loop over Vertexes of Particle B */
           for ( uint j=0; j<num_vertex_B; j++ )
	       {

               /* _|_ distance between vertex and plane */
               vertex_dis = dot( Faces_A[i].normal,( Vertex_B[j] - Faces_A[i].centroid) );

               /* if we past the plane report collision */
	           if( vertex_dis <= 0.00000f )
	           {
	        	   is_contact = true;
	        	   break;
	           }

	        }/* End of Vertexes of Particle B */


             /* Found SP so exit */
             if( !is_contact )
             {
               if(SimParms.use_hist)
               {
                face_sp[0] = i;
               }
	           return true;
             }

          }/* end of COM check */

	  } /* End of checking all planes of A */


     return false;
}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
     /* (6) Quick Check for Vertex-Face SP O( F1V2 + F2V1 )= O(2FV)*/
/*---------------------------------------------------------------------------*/
__device__ bool is_SP_Face( Plane Faces_A,float3 Pos_B, uint num_vertex_B, float3 *Vertex_B)
{
   bool   is_contact;
   float  vertex_dis ;

/*---------------------------------------------------------------------------*/
/*                     TYPE 1: PLANE SEARCH FACE A                           */
/*---------------------------------------------------------------------------*/

         is_contact = false; /* check for intersection */

         if( dot( Faces_A.normal, ( Pos_B - Faces_A.centroid ) )>0.0 )
         {
           /* Loop over Vertexes of Particle B */
           for ( uint j=0; j<num_vertex_B; j++ )
	       {

               /* _|_ distance between vertex and plane */
               vertex_dis = dot( Faces_A.normal,( Vertex_B[j] - Faces_A.centroid) );

               /* if we past the plane report collision */
	           if( vertex_dis <= 0.00000f )
	           {
	        	   is_contact = true;
	        	   break;
	           }

	        }/* End of Vertexes of Particle B */


             /* Found SP so exit */
             if( !is_contact )
             {
	           return true;
             }

          }/* end of COM check */



     return false;
}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
     /* (6) Quick Check for Vertex-Face SP O( F1V2 + F2V1 )= O(2FV)*/
/*---------------------------------------------------------------------------*/
__device__ bool is_SP_Poly_VolumeObject( uint num_faces_Poly, Plane *Faces_Poly,
		                         float3 Pos_DObject, uint num_vertex_DObject, float3 *Vertex_DObject )
{
   bool is_contact;
   float  vertex_dis ;

/*---------------------------------------------------------------------------*/
/*                     TYPE 1: PLANE SEARCH FACE A                           */
/*---------------------------------------------------------------------------*/

      /* Loop over Planes of Particle A */
	  for ( uint i=0; i<num_faces_Poly; i++)
	  {
         is_contact = false; /* check for intersection */





         if( dot( Faces_Poly[i].normal, ( Pos_DObject - Faces_Poly[i].centroid ) )>0.0 )
         {

        	// printf(" %f %f %f \n",Faces_Poly[i].centroid.x,Faces_Poly[i].centroid.y,Faces_Poly[i].centroid.z);
           /* Loop over Vertexes of Particle B */
           for ( uint j=0; j<num_vertex_DObject; j++ )
	       {


               /* _|_ distance between vertex and plane */
               vertex_dis = dot( Faces_Poly[i].normal,( Vertex_DObject[j] - Faces_Poly[i].centroid) );

               /* if we past the plane report collision */
	           if( vertex_dis <= 0.00000f )
	           {
	        	   is_contact =true;
	        	   break;
	           }

	        }/* End of Vertexes of Particle B */


             /* Found SP so exit */
             if( !is_contact )
             {

	           return true;
             }

          }/* end of COM check */

	  } /* End of checking all planes of A */


     return false;
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
     /* (6) Quick Check for Vertex-Face SP O( F1V2 + F2V1 )= O(2FV)*/
/*---------------------------------------------------------------------------*/
__device__ bool is_SP_VolumeObject_Poly( uint num_faces_DObject, Surface *Faces_DObject,
		                         float3 Pos_B, uint num_vertex_B, float3 *Vertex_B )
{
   bool is_contact;
   float  vertex_dis ;

/*---------------------------------------------------------------------------*/
/*                     TYPE 1: PLANE SEARCH FACE A                           */
/*---------------------------------------------------------------------------*/

      /* Loop over Planes of Particle A */
	  for ( uint i=0; i<num_faces_DObject; i++)
	  {
         is_contact = false; /* check for intersection */

         if( dot( Faces_DObject[i].normal, ( Pos_B - Faces_DObject[i].centroid ) )>0.0 )
         {
           /* Loop over Vertexes of Particle B */
           for ( uint j=0; j<num_vertex_B; j++ )
	       {

               /* _|_ distance between vertex and plane */
               vertex_dis = dot( Faces_DObject[i].normal,( Vertex_B[j] - Faces_DObject[i].centroid) );

               /* if we past the plane report collision */
	           if( vertex_dis <= 0.00000f )
	           {
	        	   is_contact =true;
	        	   break;
	           }

	        }/* End of Vertexes of Particle B */


             /* Found SP so exit */
             if( !is_contact )
             {
	           return true;
             }

          }/* end of COM check */

	  } /* End of checking all planes of A */


     return false;
}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
                  /* Return the grid position given the HASH*/
/*---------------------------------------------------------------------------*/
__device__ int3 Decode(int hash)
{
  int3 hash_dec;

  hash_dec.x = hash%(SimParms.num_NNCells.x*SimParms.num_NNCells.y)%SimParms.num_NNCells.x;
  hash_dec.z = (hash%(SimParms.num_NNCells.x*SimParms.num_NNCells.y))/SimParms.num_NNCells.x;
  hash_dec.y = hash/(SimParms.num_NNCells.x*SimParms.num_NNCells.y);

  return hash_dec;
}
/*-----------------------------------------------------------------------------*/




/*-----------------------------------------------------------------------------*/
        /* 1D-Sparse->1D mapping ( gets memory locations based on PreFix Sum)
              *     (uses existing device arrays)
              * 1) 1D Sparse  Matrix: dInput)
                2) 1D Compact Matrix: dOutput  */
/*-----------------------------------------------------------------------------*/

void SpatialDecomp_ThrustPrefixSum_1DSparse(uint *dSprase1D, uint* dCompact1D_R,
		                                                     uint NumParticles )
{
  thrust::inclusive_scan( thrust::device_ptr<uint>( dSprase1D ),
		                  thrust::device_ptr<uint>( dSprase1D + NumParticles ),
		                  thrust::device_ptr<uint>( dCompact1D_R)             );
}
/*-----------------------------------------------------------------------------*/





/*-----------------------------------------------------------------------------*/
                    /* DEBUG METHODS TO PRINT DATA */
/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__device__ void PrintVectorD(float3 Vec)
{
  printf(" %f   %f  %f \n", Vec.x,Vec.y,Vec.z);
}

__device__ void PrintVectorND(float3 Vec)
{
  printf("  %+10.5f %+10.5f %+10.5f  ", Vec.x,Vec.y,Vec.z);
}


__device__ void PrintVectorEND(float3 Vec)
{
  printf("  %+6.4E  %+6.4E  %+6.4E  ", Vec.x,Vec.y,Vec.z);
}

__device__ void PrintVectorI(int3 Vec)
{
  printf(" %d  %d  %d \n", Vec.x,Vec.y,Vec.z);
}

/*-----------------------------------------------------------------------------*/

__device__ void PrintQuartD(Quaterion Vec)
{
  printf(" ( %f , %f , %f  %f ) \n", Vec.w, Vec.x,Vec.y,Vec.z);
}
/*-----------------------------------------------------------------------------*/




/*-----------------------------------------------------------------------------*/
__device__ void PrintMatrix3(float3 *M)
{

  printf("  %f , %f , %f  \n", M[0].x, M[0].y, M[0].z);
  printf("  %f , %f , %f  \n", M[1].x, M[1].y, M[1].z);
  printf("  %f , %f , %f  \n", M[2].x, M[2].y, M[2].z);

}









/*___________________________________________________________________________*/
                     /* Rotate Current Object */
/*___________________________________________________________________________*/
__device__ POBJ_ROT RotatePolyhedra( uint P_type, float3 Pos_P,
		                             bool rotate_flag,Quaterion position_ornt )
{
	POBJ_ROT POBJ_RT ;
    if(rotate_flag)
	{
		 Quaterion Q ;
		 Quaterion QT;

		 Q  = position_ornt;
		 QT = conjugateD(Q);

		 /* Rotate current particle */
		 for( uint i=0; i< max( ParticleObject[P_type].num_faces,
								ParticleObject[P_type].num_vertex); i++ )
		 {

		   if(i<ParticleObject[P_type].num_vertex)
		   {
			  POBJ_RT.Vertex[i]  = make_vectorD( ( Q*make_quaterionD
							( ParticleObject[P_type ].Local_Vertex[i]) * QT ) )  + Pos_P;
		   }

		   if(i<ParticleObject[P_type].num_faces)
		   {
			 POBJ_RT.face[i].normal   = make_vectorD( ( Q*make_quaterionD (
							  ParticleObject[P_type].face[i].normal )  *QT ) );

			 POBJ_RT.face[i].centroid = make_vectorD( ( Q*make_quaterionD (
							  ParticleObject[P_type].face[i].centroid
							   - ParticleObject[P_type].COM )*QT ) )
								+ Pos_P            ;
		   }

		 }

	  }
	  else
	      {
	        for( uint i=0; i< max( ParticleObject[ P_type ].num_vertex,
	        		               ParticleObject[ P_type ].num_faces); i++ )
	        {
	     	   if( i < ParticleObject[ P_type ].num_vertex)
	     	   {
	             POBJ_RT.Vertex[i]  =  ParticleObject[ P_type ].Local_Vertex[i] + Pos_P;
	     	   }

	    	   if( i < ParticleObject[P_type].num_faces )
	    	   {
	             POBJ_RT.face[i].normal   = ParticleObject[ P_type ].face[i].normal ;
	             POBJ_RT.face[i].centroid = ParticleObject[ P_type ].face[i].centroid - ParticleObject[P_type].COM  + Pos_P;
	    	   }
	        }
	    }



	    return POBJ_RT;

}
/*___________________________________________________________________________*/


/*___________________________________________________________________________*/
                      /* Return Inertia*Force */
/*___________________________________________________________________________*/
__device__ float3 Get_Angular_Acc( uint p_index, uint p_type, float3 forceL,
		                           Quaterion position_ornt, float3 velocity_ang = make_float3(0.0f))
{
  /*-----------------------------------------------------------------------------*/
	  	             /* Get current inertia tensor */
  /*-----------------------------------------------------------------------------*/

    Quaterion Q  = position_ornt;
    Quaterion QT = conjugateD(Q);

    float IC [9];
    float RQ [9];

    /* Get current rotation matrix from Orientation Quart */
    RQ[0] = ( Q.w*Q.w + Q.x*Q.x - Q.y*Q.y - Q.z*Q.z);
    RQ[1] = ( 2.0f*Q.x*Q.y - 2.0f*Q.w*Q.z);
	RQ[2] = ( 2.0f*Q.x*Q.z + 2.0f*Q.w*Q.y );
	RQ[3] = ( 2.0f*Q.x*Q.y + 2.0f*Q.w*Q.z );
	RQ[4] = ( Q.w*Q.w - Q.x*Q.x + Q.y*Q.y - Q.z*Q.z);
	RQ[5] = ( 2.0f*Q.y*Q.z - 2.0f*Q.w*Q.x )  ;
	RQ[6] = ( 2.0f*Q.x*Q.z -2.0f*Q.w*Q.y );
	RQ[7] = ( 2.0f*Q.y*Q.z + 2.0f*Q.w*Q.x );
    RQ[8] = ( Q.w*Q.w - Q.x*Q.x - Q.y*Q.y + Q.z*Q.z);

    /* R(t)xI(0) */
	for (unsigned int i = 0; i < 3; i++)
	{
	   for (unsigned int j = 0; j < 3; j++)
	   {
	  		  float sum = 0;

	  		  for (unsigned int k = 0; k < 3; k++)
	  		  {
	  		      sum += ( RQ[ i*3 + k ] * (ParticleObject[p_type].InertiaT [ k*3 + j ]));

	  		  }

	  		  IC[i*3 + j] = sum;
	    }
	 }


	 float RQT[9];

	 RQT[0] = RQ[0] ;
	 RQT[1] = RQ[3] ;
	 RQT[2] = RQ[6];

	 RQT[3] = RQ[1] ;
	 RQT[4] = RQ[4];
	 RQT[5] = RQ[7];

	 RQT[6] = RQ[2] ;
	 RQT[7] = RQ[5] ;
	 RQT[8] = RQ[8] ;


	 /* R(t)xI(0)xRT(t) */
	    float IN[9];
	 for ( unsigned int i = 0; i < 3; i++ )
	 {
	  	 for (unsigned int j = 0; j < 3; j++)
	     {
	  	    float sum = 0;

	  		for (unsigned int k = 0; k < 3; k++)
	  		{
	  		      sum += ( (IC[ k*3 + j ])*RQT[ i*3 + k ]);

	  		}

	  		IN[i*3 + j] = sum;
	     }
	 }


//	    printf("In\n");
//	    printf("%f %f %f \n",IN[0],IN[1],IN[2]);
//	    printf("%f %f %f \n",IN[3],IN[4],IN[5]);
//	    printf("%f %f %f \n",IN[6],IN[7],IN[8]);
//	    printf("\n");

     /* Get the resultant angular acceleration and orientation */
	 float angA[3];
	 float HLV[3];
	 float3 H;


	 float forceLV[3];

	 forceLV[0] = forceL.x;
	 forceLV[1] = forceL.y;
	 forceLV[2] = forceL.z;

	 float velocity_angLV[3];

	 velocity_angLV[0] = velocity_ang.x;
	 velocity_angLV[1] = velocity_ang.y;
	 velocity_angLV[2] = velocity_ang.z;

     /* Get angular acceleration */
	 for ( unsigned int i = 0; i < 3; i++ )
	 {
	  	float sum = 0;
	  	float sum_H = 0;

	  	for ( unsigned int k = 0; k < 3; k++ )
	  	{
	  		sum += (IN[ i*3 + k ] * (forceLV[ k ]));
	  		sum_H += (IN[ i*3 + k ] * (velocity_angLV[ k ]));

	  	}
	    angA[i] = sum;
	    HLV[i] = sum_H;
	  }
	  H = make_float3(HLV[0],HLV[1],HLV[2]);
	  H = cross(velocity_ang,H);

	  /* Update Dynamics Information EQ 7 */
	  return make_float3(angA[0],angA[1],angA[2]);


}


__device__ EulerMoment Get_EulerMomentTerms( uint p_index, uint p_type, float3 forceL,
		                           Quaterion position_ornt, float3 velocity_ang = make_float3(0.0f))
{
  /*-----------------------------------------------------------------------------*/
	  	             /* Get current inertia tensor */
  /*-----------------------------------------------------------------------------*/

    Quaterion Q  = position_ornt;
    Quaterion QT = conjugateD(Q);

    float IC [9];
    float RQ [9];

    /* Get current rotation matrix from Orientation Quart */
    RQ[0] = ( Q.w*Q.w + Q.x*Q.x - Q.y*Q.y - Q.z*Q.z);
    RQ[1] = ( 2.0f*Q.x*Q.y - 2.0f*Q.w*Q.z);
	RQ[2] = ( 2.0f*Q.x*Q.z + 2.0f*Q.w*Q.y );
	RQ[3] = ( 2.0f*Q.x*Q.y + 2.0f*Q.w*Q.z );
	RQ[4] = ( Q.w*Q.w - Q.x*Q.x + Q.y*Q.y - Q.z*Q.z);
	RQ[5] = ( 2.0f*Q.y*Q.z - 2.0f*Q.w*Q.x )  ;
	RQ[6] = ( 2.0f*Q.x*Q.z -2.0f*Q.w*Q.y );
	RQ[7] = ( 2.0f*Q.y*Q.z + 2.0f*Q.w*Q.x );
    RQ[8] = ( Q.w*Q.w - Q.x*Q.x - Q.y*Q.y + Q.z*Q.z);

    /* R(t)xI(0) */
	for (unsigned int i = 0; i < 3; i++)
	{
	   for (unsigned int j = 0; j < 3; j++)
	   {
	  		  float sum = 0;

	  		  for (unsigned int k = 0; k < 3; k++)
	  		  {
	  		      sum += ( RQ[ i*3 + k ] * (ParticleObject[p_type].InertiaT [ k*3 + j ]));

	  		  }

	  		  IC[i*3 + j] = sum;
	    }
	 }


	 float RQT[9];

	 RQT[0] = RQ[0] ;
	 RQT[1] = RQ[3] ;
	 RQT[2] = RQ[6];

	 RQT[3] = RQ[1] ;
	 RQT[4] = RQ[4];
	 RQT[5] = RQ[7];

	 RQT[6] = RQ[2] ;
	 RQT[7] = RQ[5] ;
	 RQT[8] = RQ[8] ;


	 /* R(t)xI(0)xRT(t) */
	    float IN[9];
	 for ( unsigned int i = 0; i < 3; i++ )
	 {
	  	 for (unsigned int j = 0; j < 3; j++)
	     {
	  	    float sum = 0;

	  		for (unsigned int k = 0; k < 3; k++)
	  		{
	  		      sum += ( (IC[ k*3 + j ])*RQT[ i*3 + k ]);

	  		}

	  		IN[i*3 + j] = sum;
	     }
	 }


//	    printf("In\n");
//	    printf("%f %f %f \n",IN[0],IN[1],IN[2]);
//	    printf("%f %f %f \n",IN[3],IN[4],IN[5]);
//	    printf("%f %f %f \n",IN[6],IN[7],IN[8]);
//	    printf("\n");

     /* Get the resultant angular acceleration and orientation */
	 float angA[3];
	 float HLV[3];
	 float3 H;


	 float forceLV[3];

	 forceLV[0] = forceL.x;
	 forceLV[1] = forceL.y;
	 forceLV[2] = forceL.z;

	 float velocity_angLV[3];

	 velocity_angLV[0] = velocity_ang.x;
	 velocity_angLV[1] = velocity_ang.y;
	 velocity_angLV[2] = velocity_ang.z;

     /* Get angular acceleration */
	 for ( unsigned int i = 0; i < 3; i++ )
	 {
	  	float sum = 0;
	  	float sum_H = 0;

	  	for ( unsigned int k = 0; k < 3; k++ )
	  	{
	  		sum += (IN[ i*3 + k ] * (forceLV[ k ]));
	  		sum_H += (IN[ i*3 + k ] * (velocity_angLV[ k ]));

	  	}
	    angA[i] = sum;
	    HLV[i] = sum_H;
	  }
	  H = make_float3(HLV[0],HLV[1],HLV[2]);
	  H = cross(velocity_ang,H);


	  EulerMoment Result;
	  Result.ang_acc = make_float3(angA[0],angA[1],angA[2]);
	  Result.wxH = H;

	  /* Update Dynamics Information EQ 7 */
	  return Result;


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

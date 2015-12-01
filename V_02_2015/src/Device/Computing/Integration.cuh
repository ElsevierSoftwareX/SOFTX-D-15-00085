

/*---------------------------------------------------------------------------*/
/* (5) */
/*---------------------------------------------------------------------------*/
__global__ void Integrate_Euler_NonSymmetry(
		                       float3    *force_com_PP,
		                       float3    *force_ang_PP,
		                       float3    *force_com_wall,
		                       float3    *force_ang_wall,
		                       float3    *force_com_lifter,
		                       float3    *force_ang_lifter,

		                       float3    *position_com,
		                       Quaterion *position_ornt,
		                       float3    *velocity_com,
		                       float3    *velocity_ang,
		                       uint      *P_ObjectType,
		                       int       *P_ID,
		                       uint       Num_Particles )
{

    uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

	if( (index < Num_Particles) && P_ID[index]>-1 )
	{



	float delta = SimParms.InitalDelta_t;


	/* acc is in cm so F/m= (N/kg)*/
    float Mass = ParticleObject[P_ObjectType[index]].mass;


    /* 1. Net acceleration acting on particle for this step */
    float3 accelrat = (force_com_wall[index]+force_com_PP[index]);


    if(Num_DynamicObjects>0)
    {
    	accelrat += force_com_lifter[index] ;
    }

    accelrat*=(1.0f/Mass);


    /* set forces to zero */
    force_com_PP    [index] = make_float3(0.0f);


	/* 2. Integrate the Translational Velocity "cm/s" a(t-1) + a(t) */
    velocity_com [index]  += (accelrat)*SimParms.InitalDelta_t;



    position_com[index] += velocity_com[index]*delta;


    /* If we unit testing print all details */
    if(SimParms.unit_test==2)
    {
       if(index==0)
       {
    	 for (int i=0; i<Num_Particles; i++)
    	 {
    	   printf(" Particle %d EK %f AngEK %f \n",i,0.5*Mass*velocity_com[i]*velocity_com[i], 0.0f);
    	   PrintVectorND(position_com[i]);
    	   PrintVectorND(velocity_com[i]);

    	   if(SimParms.particle_type==polyhedra)
    	   {
    		   PrintQuartND(position_ornt[i]);
    	   }
    	   PrintVectorD(velocity_ang[i]);

    	 }

       }
    }

    if(SimParms.Mode2D)
    {
      velocity_com[index].z = 0.0f;
    }



    float3 forceL = force_ang_PP[index] + force_ang_wall[index];

    if( Num_DynamicObjects > 0 )
    {
    	forceL += force_ang_lifter[index];
    }

    /* set forces to zero */
    force_ang_PP    [index] = make_float3(0.0f);



    if( SimParms.Rotation )
    {
      /* Compute acceleration and update angular velocity */
      if( dot(forceL,forceL) > 0.0f )
      {


 	     /* Collision Response Rotation */
          float3 ang_acc;

          if(SimParms.particle_type==0)
          {
             ang_acc   = ParticleObject[P_ObjectType[index]].InertiaT[0]*forceL;
          }
          else
          {
            ang_acc = Get_Angular_Acc(index,P_ObjectType[index],forceL,position_ornt[index]);
            // Additional Accuracy Get_EulerMomentTerms
          }


          //PrintVectorD(ang_acc);

          velocity_ang [index]  += ((ang_acc*SimParms.InitalDelta_t));

          /* Rolling resistance */
          velocity_ang[index] -= velocity_ang[index]*SimParms.Roll_Res;
      }


      if(SimParms.particle_type==1 || SimParms.sphere_orient)
      {
	    float3   omega    = velocity_ang[index];
        float    magOmega = dot(omega,omega);

        Quaterion dQ = make_IquaterionD();

        if( magOmega > 0.000000f )
        {


      	    magOmega     = sqrt(magOmega);
      	    float3 dir   = omega/magOmega;
            dQ.w         = cos(0.5000f*magOmega*delta);
            double sinv  = sin(0.5000f*magOmega*delta);
            dQ.x         = sinv*dir.x;
            dQ.y         = sinv*dir.y;
            dQ.z         = sinv*dir.z;
        }
         /* load current orientation */
	     Quaterion C_Ornt       = position_ornt[index];
	     Quaterion Q            = normaliseD((dQ*C_Ornt));/* Integrate */

	     /* Check for singularity at 360% */
	     if( Q.w <= -1.00000f)
	     {
	    	 position_ornt[ index ] = dQ;
	     }
	     else
	     {
	      /* 2. Integrate the Angular Position */
	      position_ornt[ index ] = Q;
	     }

	   	if(SimParms.Mode2D)
	   	{
	   	   position_ornt[ index ].x = 0.0f;
	   	   position_ornt[ index ].y = 0.0f;
	   	}

      }/* End  */

  	  if(SimParms.Mode2D)
  	  {
  	    velocity_ang [index].x   = 0.0f;
  	    velocity_ang [index].y   = 0.0f;
  	  }

     } /* End if rotation */

   }/* End particles */

}
/*---------------------------------------------------------------------------*/



/*___________________________________________________________________________*/

__global__ void Integrate_Euler_Symmetry( float     *force_com_X,
		                                  float     *force_com_Y,
		                                  float     *force_com_Z,
		                                  float     *force_ang_X,
		                                  float     *force_ang_Y,
		                                  float     *force_ang_Z,

		                                  float3    *force_com_wall,
										  float3    *force_ang_wall,
										  float3    *force_com_lifter,
										  float3    *force_ang_lifter,

										  float3    *position_com,
										  Quaterion *position_ornt,
										  float3    *velocity_com,
										  float3    *velocity_ang,
										  uint      *P_ObjectType,
										  int       *P_ID,
										  uint       Num_Particles )
{

    uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

	if( index < Num_Particles && P_ID[index]>-1 )
	{

		float delta = SimParms.InitalDelta_t;


		/* acc is in cm so F/m= (N/kg)*/
	    float Mass = ParticleObject[P_ObjectType[index]].mass;

	    float3 force_com_PP = make_float3( force_com_X[index], force_com_Y[index], force_com_Z[index]);

	    float3 force_ang_PP = make_float3( force_ang_X[index], force_ang_Y[index], force_ang_Z[index]);

	    /* 1. Net acceleration acting on particle for this step */
	    float3 accelrat = (force_com_wall[index]+force_com_PP);


	    if( Num_DynamicObjects > 0 )
	    {
	    	accelrat += force_com_lifter[index] ;
	    }

	    accelrat*=(1.0f/Mass);

		/* 2. Integrate the Translational Velocity "cm/s" a(t-1) + a(t) */
	    velocity_com [index]  += (accelrat)*SimParms.InitalDelta_t;

	    /* 2. Integrate Position */
	    position_com[index] += velocity_com[index]*delta;


	    if(SimParms.Mode2D)
	    {
	      velocity_com[index].z=0.0f;
	    }



	    float3 forceL = force_ang_PP + force_ang_wall[index];

	    if(Num_DynamicObjects>0)
	    {
	    	forceL += force_ang_lifter[index];
	    }

	    /* Set PP forces to zero */
		 force_com_X[index]=0.0f;
		 force_com_Y[index]=0.0f;
		 force_com_Z[index]=0.0f;

		 force_ang_X[index]=0.0f;
		 force_ang_Y[index]=0.0f;
		 force_ang_Z[index]=0.0f;


	    if( SimParms.Rotation )
	    {
	      /* Compute acceleration and update angular velocity */
	      if( dot(forceL,forceL) > 0.0f )
	      {
	 	     /* Collision Response Rotation */
	          float3 ang_acc;

	          if(SimParms.particle_type==0)
	          {
	             ang_acc   = ParticleObject[P_ObjectType[index]].InertiaT[0]*forceL;
	          }
	          else
	          {
	            ang_acc = Get_Angular_Acc(index,P_ObjectType[index],forceL,position_ornt[index]);
	          }

	          velocity_ang [index]  += ((ang_acc*SimParms.InitalDelta_t));

	          /* Rolling resistance */
	          velocity_ang[index] -= velocity_ang[index]*SimParms.Roll_Res;
	      }


	      if(SimParms.particle_type==1 || SimParms.sphere_orient)
	      {
		    float3   omega    = velocity_ang[index];
	        float    magOmega = dot(omega,omega);

	        Quaterion dQ = make_IquaterionD();

	        if( magOmega > 0.000000f )
	        {


	      	    magOmega     = sqrt(magOmega);
	      	    float3 dir   = omega/magOmega;
	            dQ.w         = cos(0.5000f*magOmega*delta);
	            double sinv  = sin(0.5000f*magOmega*delta);
	            dQ.x         = sinv*dir.x;
	            dQ.y         = sinv*dir.y;
	            dQ.z         = sinv*dir.z;
	        }
	         /* load current orientation */
		     Quaterion C_Ornt       = position_ornt[index];
		     Quaterion Q            = normaliseD((dQ*C_Ornt));/* Integrate */

		     /* Check for singularity at 360% */
		     if( Q.w <= -1.00000f)
		     {
		    	 position_ornt[ index ] = dQ;
		     }
		     else
		     {
		      /* 2. Integrate the Angular Position */
		      position_ornt[ index ] = Q;
		     }

		   	if(SimParms.Mode2D)
		   	{
		   	   position_ornt[ index ].x = 0.0f;
		   	   position_ornt[ index ].y = 0.0f;
		   	}

	      }/* End  */

	  	  if(SimParms.Mode2D)
	  	  {
	  	    velocity_ang [index].x   = 0.0f;
	  	    velocity_ang [index].y   = 0.0f;
	  	  }

	     } /* End if rotation */

   }/* End particles */

}



/*---------------------------------------------------------------------------*/
           /* kernels that still need to be tested/fully implemented */
/*---------------------------------------------------------------------------*/

__global__ void Integrate_Verlet_NonSymmetryInit(
		                       float3    *force_com,
		                       float3    *force_ang,
		                       float3    *force_com_wall,
		                       float3    *force_com_lifter,
		                       float3    *position_com_old,

		                       float3    *position_com,
		                       Quaterion *position_ornt,
		                       float3    *velocity_com,
		                       float3    *velocity_ang,
		                       float3    *accelrat_com,

		                       uint      *P_ObjectType,
		                       int       *P_ID              )
{

    uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

	if( index < SimParms.Num_Particles && P_ID[index]>-1 )
	{
		float delta = SimParms.InitalDelta_t;


			/* acc is in cm so F/m= (N/kg)*/
		    float Mass = ParticleObject[P_ObjectType[index]].mass;


		    /* 1. Net acceleration acting on particle for this step */
		    float3 accelrat = (force_com_lifter[index] + force_com_wall[index]+force_com[index])*(1.0f/Mass);

		    if(SimParms.Mode2D)
		    {
		         accelrat.z=0.0f;
		    }


		    float3 c_pos = position_com[index];

		    /* 2. Integrate position */
		    position_com[index] = c_pos + velocity_com[index]*delta +0.50f*accelrat*delta*delta;


			/* 3. Integrate the Translational Velocity "cm/s" a(t-1) + a(t) */
		    velocity_com [index] = (position_com[index]-c_pos)*(1.0f/delta) + accelrat*delta;

		    position_com_old[index] = c_pos;

		    //printf("%f %f \n",position_com[index].y,velocity_com[index].y);

		    if(SimParms.Mode2D)
		    {
		       velocity_com[index].z=0.0f;
		    }



		    float3 forceL = force_ang[index];

		    if(SimParms.Rotation)
		    {
		      if(dot(forceL,forceL)>0.0f )
		      {

		   	      /* Collision Response Rotation */
		          float3 ang_acc;

		          if(SimParms.particle_type==0)
		          {
		        	  float R_A = ParticleObject[P_ObjectType[index] ].radius;
		              ang_acc = ( (1.0f/(0.40f*R_A*R_A*Mass)) )*forceL;
		          }
		          else
		          {
		           ang_acc = Get_Angular_Acc(index,P_ObjectType[index],forceL,position_ornt[index]);
		          }

		          velocity_ang [index]  += ((ang_acc*SimParms.InitalDelta_t));

		      }

		      if(SimParms.particle_type==1)
		      {
			    float3   omega    = velocity_ang[index];
		        float    magOmega = dot(omega,omega);



		        Quaterion dQ = make_IquaterionD();

		        if( magOmega > 0.000000f )
		        {


		      	    magOmega     = sqrt(magOmega);
		      	    float3 dir   = omega/magOmega;
		            dQ.w         = cos(0.5000f*magOmega*delta);
		            double sinv  = sin(0.5000f*magOmega*delta);
		            dQ.x         = sinv*dir.x;
		            dQ.y         = sinv*dir.y;
		            dQ.z         = sinv*dir.z;
		        }
		         /* load current orientation */
			     Quaterion C_Ornt       = position_ornt[index];
			     Quaterion Q            = normaliseD((dQ*C_Ornt));/* Integrate */

			     velocity_ang[index]-=velocity_ang[index]*SimParms.Roll_Res;

			     /* Check for singularity at 360% */
			     if( Q.w <= -1.00000f)
			     {
			    	 position_ornt[ index ] = dQ;
			     }
			     else
			     {
			      /* 2. Integrate the Angular Position */
			      position_ornt[ index ] = Q;
			     }

			     //PrintQuartD(position_ornt[ index_A ]);

			   	if(SimParms.Mode2D)
			   	{
			   	   position_ornt[ index ].x = 0.0f;
			   	   position_ornt[ index ].y = 0.0f;
			   	}
		      }/* End if poly */

		  	  if(SimParms.Mode2D)
		  	  {
		  	    velocity_ang [index].x   = 0.0f;
		  	    velocity_ang [index].y   = 0.0f;
		  	  }

		    } /* End if rotation */

		    /* set wall force */
		    force_com_wall  [index] = make_float3(0.0f);
		    force_com_lifter[index] = make_float3(0.0f);
	}
}





__global__ void Integrate_Verlet_NonSymmetry(
		                       float3    *force_com,
		                       float3    *force_ang,
		                       float3    *force_com_wall,
		                       float3    *force_com_lifter,
		                       float3    *position_com_old,

		                       float3    *position_com,
		                       Quaterion *position_ornt,
		                       float3    *velocity_com,
		                       float3    *velocity_ang,
		                       float3    *accelrat_com,

		                       uint      *P_ObjectType,
		                       int       *P_ID              )
{

    uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

	if( index < SimParms.Num_Particles && P_ID[index]>-1 )
	{



	float delta = SimParms.InitalDelta_t;


	/* acc is in cm so F/m= (N/kg)*/
    float Mass = ParticleObject[P_ObjectType[index]].mass;


    /* 1. Net acceleration acting on particle for this step */
    float3 accelrat = (force_com_lifter[index] + force_com_wall[index]+force_com[index])*(1.0f/Mass);



    if(SimParms.Mode2D)
    {
         accelrat.z=0.0f;
    }

    float3 c_pos = position_com[index];

    /* 2. Integrate position */
    position_com[index] = 2.0f*c_pos -position_com_old[index] +accelrat*delta*delta;



	/* 3. Integrate the Translational Velocity "cm/s" a(t-1) + a(t) */
    velocity_com [index] = (1.0f/(2.0f*delta))*(position_com[index]-position_com_old[index]) +accelrat*delta;

    position_com_old[index] = c_pos;

    if(SimParms.Mode2D)
    {
       velocity_com[index].z=0.0f;
    }



    float3 forceL = force_ang[index];

    if(SimParms.Rotation)
    {
      if(dot(forceL,forceL)>0.0f )
      {

   	      /* Collision Response Rotation */
          float3 ang_acc;

          if(SimParms.particle_type==0)
          {
        	  float R_A = ParticleObject[P_ObjectType[index] ].radius;
              ang_acc = ( (1.0f/(0.40f*R_A*R_A*Mass)) )*forceL;
          }
          else
          {
           ang_acc = Get_Angular_Acc(index,P_ObjectType[index],forceL,position_ornt[index]);
          }

          velocity_ang [index]  += ((ang_acc*SimParms.InitalDelta_t));

      }

      if(SimParms.particle_type==1)
      {
	    float3   omega    = velocity_ang[index];
        float    magOmega = dot(omega,omega);



        Quaterion dQ = make_IquaterionD();

        if( magOmega > 0.000000f )
        {


      	    magOmega     = sqrt(magOmega);
      	    float3 dir   = omega/magOmega;
            dQ.w         = cos(0.5000f*magOmega*delta);
            double sinv  = sin(0.5000f*magOmega*delta);
            dQ.x         = sinv*dir.x;
            dQ.y         = sinv*dir.y;
            dQ.z         = sinv*dir.z;
        }
         /* load current orientation */
	     Quaterion C_Ornt       = position_ornt[index];
	     Quaterion Q            = normaliseD((dQ*C_Ornt));/* Integrate */

	     velocity_ang[index]-=velocity_ang[index]*SimParms.Roll_Res;

	     /* Check for singularity at 360% */
	     if( Q.w <= -1.00000f)
	     {
	    	 position_ornt[ index ] = dQ;
	     }
	     else
	     {
	      /* 2. Integrate the Angular Position */
	      position_ornt[ index ] = Q;
	     }

	     //PrintQuartD(position_ornt[ index_A ]);

	   	if(SimParms.Mode2D)
	   	{
	   	   position_ornt[ index ].x = 0.0f;
	   	   position_ornt[ index ].y = 0.0f;
	   	}
      }/* End if poly */

  	  if(SimParms.Mode2D)
  	  {
  	    velocity_ang [index].x   = 0.0f;
  	    velocity_ang [index].y   = 0.0f;
  	  }

    } /* End if rotation */

    /* set wall force */
    force_com_wall  [index] = make_float3(0.0f);
    force_com_lifter[index] = make_float3(0.0f);



	}/* End particles */
}




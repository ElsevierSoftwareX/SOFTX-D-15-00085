/*---------------------------------------------------------------------------*/
/*          (1)       Force Model for Particle Surface Contact               */
/*---------------------------------------------------------------------------*/
__device__ Forces Collision_Response_Particle_StaticSurface ( float3 PVel,
		                                                float3 normal,
		                                                float  pen_dis,
		                                                float3 R_Cpoint_COM,
		                                                float3 velocity_ang ,
		                                                POBJ   PData   )
{
    Forces Force;

    float3 Vnormal  = dot(PVel, normal)*normal;

    /* 1. Normal Reaction Force with Viscous Damping */
   float  Fnormal    =  PData.surface_Kn*pen_dis;
   float3 Fdiss =  (PData.surface_Cn)*Vnormal;

   /* Total Force */
   Force.trans  = Fnormal*normal-Fdiss;

   float3 Fdir = Force.trans/length(Force.trans);

   /* Force must not be attractive */
   if( dot(Fdir,normal) < 0 )
   {
  	 Force.trans  = Fnormal*normal;
  	 Fnormal =0.0f;
   }
   else
   {
     if (SimParms.EnergyCalc && dot(Vnormal,Vnormal)>0.0f )
     {
      /* Add to Global Energy */
  	   Fnormal = length(Fdiss)*length(Vnormal)*SimParms.InitalDelta_t;

     }
     else
     {
  	   Fnormal = 0.0f;
     }
   }

    /* 2. Tangential Force Calculation */
    float3 Vtangent     = (PVel - Vnormal) ;

	float3 TangentForce = make_float3(0.0f);

    /* Add rotational contribution to tangent velocity */
  	if(SimParms.Rotation)
  	{
  		Vtangent += cross (velocity_ang,R_Cpoint_COM);
  	}

    float  mag_Vtangent = dot(Vtangent,Vtangent);


    if( mag_Vtangent > 0.0f )
    {
    	mag_Vtangent = sqrtf(mag_Vtangent);

        float Velf = dot(PVel,PVel);/* Factors normal impact speed */

        if( Velf>0 )
        {
         Velf = sqrtf(Velf);
        }
        else
        {
        	Velf = 1.0f;
        }

		float tan_f = ( 0.050f*(PData.mass/SimParms.InitalDelta_t) )*((PData.surface_Fric_static*(mag_Vtangent/Velf)));

		TangentForce = -1.0f*tan_f*(Vtangent/mag_Vtangent);

		/* Add energy dissipated */
		    if (SimParms.EnergyCalc  )
		    {
		      atomicAdd(&Tally_EnergyDisSurf, Fnormal + (length(TangentForce)*mag_Vtangent*SimParms.InitalDelta_t) );
		    }

    }
    else
    {
       if (SimParms.EnergyCalc  )
       {
          atomicAdd(&Tally_EnergyDisSurf, Fnormal);
       }

    }

    Force.trans += TangentForce;

    /* 2. Rotational Force */
    if( SimParms.Rotation )
    {
      Force.torque = cross(R_Cpoint_COM,Force.trans);
    }
    else
    {
       Force.torque = make_float3(0.0f);
    }




    return Force;
}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*    (3)       Force Model for Particle Moving Surface Contact              */
/*---------------------------------------------------------------------------*/
__device__ Forces Collision_Response_Particle_MovingSurface( float3 normal,
															 float  pen_dis,
															 float3 R_Body_COMA,
															 float3 VelA_Ang,
															 uint   P_typeA,
															 float3 Vel_A,
															 float3 Vel_Surface,
															 float MassA         )
{
     Forces Force;


     float3 RVel  = Vel_A -Vel_Surface;

     float3 Vn  = dot(RVel, normal)*normal;


      /* 1. Normal Reaction Force with Viscous Damping */
     float  Fnormal    =  ParticleObject[P_typeA].surface_Kn*pen_dis;
     float3 Fdiss =  (ParticleObject[P_typeA].surface_Cn)*Vn;

     /* Total Force */
     Force.trans  = Fnormal*normal-Fdiss;

     float3 Fdir = Force.trans/length(Force.trans);

     /* Force must not be attractive */
     if( dot(Fdir,normal) < 0 )
     {
    	 Force.trans  = Fnormal*normal;
    	 Fnormal = 0.0f; /* We reuse the variable for energy */
     }
     else
     {
       if (SimParms.EnergyCalc && dot(Vn,Vn)>0.0f )
       {
        /* Add to Global Energy */
    	 Fnormal = length(Fdiss)*length(Vn)*SimParms.InitalDelta_t;

       }
       else
       {
    	   Fnormal = 0.0f;
       }
     }
     /* 2. Tangential contribution */


	 float3 V_CPA       = Vel_A + cross(VelA_Ang,R_Body_COMA);

	 float3 V_CPNA       = dot(V_CPA,normal)*normal;


     float3 Vt_A       = V_CPA - V_CPNA;
	 float3 Vt_B       = Vel_Surface - dot(Vel_Surface,normal)*normal;

     /* Get Relative Tangential Velocity */
     float3  Vt_Rel    = Vt_A - Vt_B;


     float   mag_Vt_Rel  = dot(Vt_Rel,Vt_Rel);

     float3 Tangential_Force = make_float3(0.0f);


	 /* If there is a relative tangential velocity calculate force On A */
	 if( mag_Vt_Rel > 0.0f )
     {

		 mag_Vt_Rel = sqrtf(mag_Vt_Rel);

        float Velf = dot(RVel,RVel);/* Factors normal impact speed */

        if(Velf>0)
        {
         Velf = sqrtf(Velf);
        }
        else
        {
        	Velf = 1.0f;
        }

		float tan_f = ParticleObject[P_typeA].surface_Fric_kinetic*(mag_Vt_Rel/Velf);

		Tangential_Force = -1.0f*tan_f*(Vt_Rel/mag_Vt_Rel)*( MassA/SimParms.InitalDelta_t)*0.050f;

		/* Add energy dissipated */
		if (SimParms.EnergyCalc  )
		{
		   atomicAdd(&Tally_EnergyDisSurf, Fnormal + (length(Tangential_Force)*mag_Vt_Rel*SimParms.InitalDelta_t) );
		}

    }
    else
    {
       if (SimParms.EnergyCalc)
       {
          atomicAdd(&Tally_EnergyDisSurf, Fnormal);
       }

    }

       Force.trans += Tangential_Force;

	   /* 3. Rotational Force */

		if(SimParms.Rotation)
		{
		  Force.torque = cross(R_Body_COMA,Force.trans);
		}
		else
		{
			Force.torque  = make_float3(0.0f);
		}


    return Force;
}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*    (3)       Force Model for Particle Dynamic Object Contact              */
/*---------------------------------------------------------------------------*/
__device__ Forces Collision_Response_Particle_VolumeObject ( float3 normal,
														float  pen_dis,
														float3 R_Body_COMA,
														float3 R_Body_COMVOBJ,
														float3 VelA_Ang,
														float3 VelVOBJ_Ang,
														uint   P_typeA,
														float3 Vel_A,
														float3 Vel_VOBJ,
														float  MassA        )
{
     Forces Force;


     float3 RVel  = Vel_A -Vel_VOBJ;

     float3 Vnormal  = dot(RVel, normal)*normal;


      /* 1. Normal Reaction Force with Viscous Damping */
     float  Fnormal    =  ParticleObject[P_typeA].Dsurface_Kn*pen_dis;
     float3 Fdiss =  (ParticleObject[P_typeA].Dsurface_Cn)*Vnormal;

     /* Total Force */
     Force.trans  = Fnormal*normal-Fdiss;

     float3 Fdir = Force.trans/length(Force.trans);

     /* Force must not be attractive */
     if( dot(Fdir,normal) < 0 )
     {
    	 Force.trans  = Fnormal*normal;
    	 Fnormal =0.0f;
     }
     else
     {
       if (SimParms.EnergyCalc && dot(Vnormal,Vnormal)>0.0f )
       {
        /* Add to Global Energy */
    	 Fnormal = length(Fdiss)*length(Vnormal)*SimParms.InitalDelta_t;

        }
       else
       {
    	   Fnormal =0.0f;
       }
     }

     /* 2. Tangential contribution */


	 float3 V_CPA      = Vel_A + cross(VelA_Ang,R_Body_COMA);
	 float3 V_CPB      = Vel_VOBJ + cross(VelVOBJ_Ang,R_Body_COMVOBJ);

	 float3 V_CPNA     = dot(V_CPA,normal)*normal;
	 float3 V_CPNB     = dot(V_CPB,normal)*normal;


     float3 Vt_A       = V_CPA - V_CPNA;
	 float3 Vt_B       = V_CPB - V_CPNB;

     /* Get Relative Tangential Velocity */
     float3  Vt_Rel    = Vt_A - Vt_B;


     float   mag_Vt_Rel  = dot(Vt_Rel,Vt_Rel);

     float3 Tangential_Force = make_float3(0.0f);


	 /* If there is a relative tangential velocity calculate force On A */
	 if( mag_Vt_Rel > 0.0f )
     {

		 mag_Vt_Rel = sqrtf(mag_Vt_Rel);

        float Velf = dot(RVel,RVel);/* Factors normal impact speed */

        if(Velf>0)
        {
         Velf = sqrtf(Velf);
        }
        else
        {
        	Velf = 1.0f;
        }

		float tan_f = ( 0.050f*(MassA/SimParms.InitalDelta_t) )*(ParticleObject[P_typeA].Dsurface_Fric_kinetic*(mag_Vt_Rel/Velf));

		Tangential_Force = -1.0f*tan_f*(Vt_Rel/mag_Vt_Rel);

		/* Add energy dissipated */
		if (SimParms.EnergyCalc  )
		{
		   atomicAdd(&Tally_EnergyDisLift, Fnormal + (length(Tangential_Force)*mag_Vt_Rel*SimParms.InitalDelta_t) );
		}

    }
    else
    {
       if (SimParms.EnergyCalc)
       {
          atomicAdd(&Tally_EnergyDisLift, Fnormal);
       }

    }

       Force.trans += Tangential_Force;

	   /* 3. Rotational Force */

		if(SimParms.Rotation)
		{
		  Force.torque = cross(R_Body_COMA,Force.trans);
		}
		else
		{
			Force.torque  = make_float3(0.0f);
		}


    return Force;
}
/*---------------------------------------------------------------------------*/






/*---------------------------------------------------------------------------*/
/*    (4)       Force Model for Sphere Dynamic Surface Contact               */
/*---------------------------------------------------------------------------*/
__device__ Forces Collision_Response_Particle_Particle( float3 RVel,
		                                                float3 normal,
		                                                float  pen_dis,
		                                                float3 R_Body_COMA,
		                                                float3 R_Body_COMB,
		                                                float3 VelA_Ang,
		                                                float3 VelB_Ang,
		                                                uint   P_typeA,
		                                                uint   P_typeB,
		                                                float3 Vel_A,
		                                                float3 Vel_B,
		                                                float  MassA,
		                                                float  MassB        )
{
     Forces Force;


     float3 Vnormal  = dot(RVel, normal)*normal;


      /* 1. Normal Reaction Force with Viscous Damping */
     float  Fnormal  =   ParticleObject[P_typeA].PP[P_typeB].Kn*pen_dis;
     float3 Fdiss    =  (ParticleObject[P_typeA].PP[P_typeB].Cn)*Vnormal;

     /* Total Normal Force */
     Force.trans     = Fnormal*normal - Fdiss;

     if (SimParms.EnergyCalc && dot(Vnormal,Vnormal)>0.0f )
     {
    	if(SimParms.use_symmetry)
    	{
          /* Add to Global Energy */
    	  Fnormal = length(Fdiss)*length(Vnormal)*SimParms.InitalDelta_t;
    	}
    	else
    	{
            /* Add to Global Energy 0.50f since we do this twice */
    		Fnormal = 0.50f*length(Fdiss)*length(Vnormal)*SimParms.InitalDelta_t;
    	}

     }
     else
     {
    	 Fnormal = 0.0f;
     }

     /* 2. Tangential contribution */

	 float3 V_CPA       = Vel_A + cross(VelA_Ang,R_Body_COMA);
	 float3 V_CPB       = Vel_B + cross(VelB_Ang,R_Body_COMB);

	 float3 V_CPNA       = dot(V_CPA,normal)*normal;
	 float3 V_CPNB       = dot(V_CPB,normal)*normal;


     float3 Vt_A       = V_CPA - V_CPNA;
	 float3 Vt_B       = V_CPB - V_CPNB;

     /* Get Relative Tangential Velocity */
     float3  Vt_Rel    = Vt_A - Vt_B;


     float   mag_Vt_Rel  = dot(Vt_Rel,Vt_Rel);

     float3 Tangential_Force = make_float3(0.0f);


	 /* If there is a relative tangential velocity calculate force On A */
	 if( mag_Vt_Rel > 0.0f )
     {

		 mag_Vt_Rel = sqrtf(mag_Vt_Rel);

        float Velf = dot(RVel,RVel);/* Factors normal impact speed */

        if(Velf>0)
        {
         Velf = sqrtf(Velf);
        }
        else
        {
        	Velf = 1.0f;
        }

		float tan_f = ( ((0.50f*(MassA+MassB))/SimParms.InitalDelta_t)*0.050f )*
				ParticleObject[P_typeA].PP[P_typeB].Fric_kinetic*(mag_Vt_Rel/Velf);

		Tangential_Force = -1.0f*tan_f*(Vt_Rel/mag_Vt_Rel);

		/* Add energy dissipated */
				if (SimParms.EnergyCalc  )
				{
				   atomicAdd(&Tally_EnergyDis_PP, Fnormal + (length(Tangential_Force)*mag_Vt_Rel*SimParms.InitalDelta_t) );
				}

	   }
	   else
	   {
		  if (SimParms.EnergyCalc)
		  {
		      atomicAdd(&Tally_EnergyDis_PP, Fnormal);
		  }

	   }

        Force.trans += Tangential_Force;

	   /* 3. Rotational Force */

		if(SimParms.Rotation)
		{
		  Force.torque = cross(R_Body_COMA,Force.trans);
		}
		else
		{
			Force.torque  = make_float3(0.0f);
		}


    return Force;
}
/*---------------------------------------------------------------------------*/





/*---------------------------------------------------------------------------*/
/*    (5)            Force Model for Drum End plates-NO Friction             */
/*---------------------------------------------------------------------------*/
__device__ Forces Collision_Response_EndPlate_Particle ( float3 RVel,
		                                                 float3 normal,
		                                                 float  pen_dis,
		                                                 float3 Point_R,
		                                                 float3 velocity_ang ,
		                                                 POBJ   PData         )
{
      Forces Force;

      float3 Vn = dot(RVel, normal)*normal;

      float3 Fdiss = PData.surface_Cn*Vn;
      float  Fn    =(PData.surface_Kn*pen_dis);

      Force.trans  = Fn*normal-Fdiss;


    /* 2. Rotational Force */
      Force.torque  = make_float3(0.0f);


    return Force;
}
/*---------------------------------------------------------------------------*/



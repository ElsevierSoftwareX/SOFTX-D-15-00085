

/* Contains the Kernels and methods that facilitate
 * the transfer between Device and Host  */


/* Structure Definitions */
/* Classification of contact Type */
enum COL_TYPE   { UNDEF       = -1,
	              SP_PLANE    =  0,
	              Vertex_Face =  1,
	              Edge_Face   =  2,
	              Face_Face   =  3,
	              Edge_Edge   =  4,
	              MEdge_MEdge =  5,
	              EdgeF_EdgeF =  6 };


/* Return type for polyhedra contact */
struct PointInPoly
{
  bool  is_point_inside;
  int   Close_Face_num [3];
  float distance   [3];
};


/* Particle Object for rotation */
struct POBJ_ROT
{
  float3 Vertex [32];
  Plane  face   [32];
};


/*------------------------------------------------------*/
             /* Contact Info */
/*------------------------------------------------------*/

/* Generic Contact Info */
struct CollData
{
  bool      is_collision;
  float     dis;
  float3    contact_point;
  float3    Normal;
  int      colltype;


  int       selected_face;
  uint      selected_vertex;
  COL_TYPE  collision_class;
};
/*------------------------------------------------------*/

struct NN_Selected_Cell
{
	uint NN_Index[32];
};

struct Forces
{
  float3 trans;
  float3 torque;
};

struct EulerMoment
{
  float3 ang_acc;
  float3 wxH;
};


/*---------------------------------------------------------------------------*/
                      /* Simulation Data */
/*---------------------------------------------------------------------------*/

__constant__  SimulationInfo     SimParms;

__constant__ WOBJ    WorldObject    [8];
__constant__ POBJ    ParticleObject [8];
__constant__ DOBJ    DynamicObject  [32];


__constant__ float3  InitVel[2];


__device__ int Num_WorldObjects;    /* Max 10 World Objects    */
__device__ int Num_ParticleObjects; /* Max 10 Particle Objects */
__device__ int Num_DynamicObjects;  /* Max 10 Particle Objects */

__device__ int Total_NN;

__device__ float fc;

/*---------------------------------------------------------------------------*/

__device__ float Tally_EnergyDis_PP;
__device__ float Tally_EnergyDisLift;
__device__ float Tally_EnergyDisSurf;
__device__ float Tally_EnergyDis_Norm;
__device__ float Tally_EnergyDis_Shear;


__device__ float Tally_EnergyTot;
/*---------------------------------------------------------------------------*/



__device__ int Kill_count;

__device__ int TotalNN=0;


/*---------------------------------------------------------------------------*/
        /* Start SIM: Sets the initial configuration values for particles */
/*---------------------------------------------------------------------------*/
__global__ void Set_Init_Config_Device(  int    num_Particle_Types,
		                                 int    num_world_Objects,
		                                 int    num_dynamic_Objects,

		                                 float  *force_com_PP_X,
		                                 float  *force_com_PP_Y,
		                                 float  *force_com_PP_Z,
		                                 float3 *force_com_PW,
		                                 float3 *force_com_PL,

		                                 float  *force_ang_PP_X,
		                                 float  *force_ang_PP_Y,
		                                 float  *force_ang_PP_Z,
		                                 float3  *force_ang_PW,
		                                 float3  *force_ang_PL,

		                                 float3  *velocity_com,
		                                 float3  *velocity_ang,
		  		                         Contact_Info    *Lifter_Contact_Hist,
		  		                         float3  *accelrat_com,
		  		                         int     *Num_PPContactHist,
		  		                         int acc_flag = 0                 )
{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;



   if( index < SimParms.Num_Particles )
   {

	 /* First thread store number of objects */
	 if( index==0 )
	 {

	   Num_WorldObjects    = num_world_Objects;
	   Num_ParticleObjects = num_Particle_Types;
	   Num_DynamicObjects  = num_dynamic_Objects;
	   /*TODO COUNTERS HERE*/
	   Total_NN = 0;
	 }
	 __syncthreads();

	 velocity_com [index]   = InitVel[0];
     velocity_ang [index]   = make_float3 (0.000000f,0.000000f,0.000000f); /*rad/sec*/

     force_com_PP_X [index] = 0.000000f;
     force_com_PP_Y [index] = 0.000000f;
     force_com_PP_Z [index] = 0.000000f;

     force_com_PW [index]   = make_float3 (0.000000f,0.000000f,0.000000f);

     force_ang_PP_X [index] = 0.000000f;
     force_ang_PP_Y [index] = 0.000000f;
     force_ang_PP_Z [index] = 0.000000f;

     force_ang_PW [index]   = make_float3 (0.000000f,0.000000f,0.000000f);

     /* If we have lifters */
     if(Num_DynamicObjects>0)
     {
        force_com_PL [index]   = make_float3 (0.000000f,0.000000f,0.000000f);
        force_ang_PL [index]   = make_float3 (0.000000f,0.000000f,0.000000f);

        Lifter_Contact_Hist[index].num_contact = 0;
        Lifter_Contact_Hist[index].cont_type = -1;
     }

     if(SimParms.use_hist)
     {
    	 Num_PPContactHist[index] =0;
     }

	 if(acc_flag==1)
	 {
	   accelrat_com [index]   = make_float3 (0.000000f,0.000000f,0.000000f);
	 }

   }

}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
        /* Start SIM: Sets the initial configuration values for particles */
/*---------------------------------------------------------------------------*/
__global__ void Set_Init_Config_Device(  int    num_Particle_Types,
		                                 int    num_world_Objects,
		                                 int    num_dynamic_Objects,
		                                 float3  *force_com_PP,
		                                 float3  *force_com_PW,
		                                 float3  *force_com_PL,
		                                 float3  *force_ang_PP,
		                                 float3  *force_ang_PW,
		                                 float3  *force_ang_PL,
		                                 float3  *velocity_com,
		                                 float3  *velocity_ang,
		  		                         Contact_Info    *Lifter_Contact_Hist,
		  		                         float3  *accelrat_com,
		  		                         int     *Num_PPContactHist,
		  		                         int acc_flag = 0                 )
{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;



   if( index < SimParms.Num_Particles )
   {

	 /* First thread store number of objects */
	 if( index==0 )
	 {

	   Num_WorldObjects    = num_world_Objects;
	   Num_ParticleObjects = num_Particle_Types;
	   Num_DynamicObjects  = num_dynamic_Objects;
	   /*TODO COUNTERS HERE*/
	   Total_NN = 0;
	 }
	 __syncthreads();

	 velocity_com [index]   = InitVel[0];
     velocity_ang [index]   = make_float3 (0.000000f,0.000000f,0.000000f); /*rad/sec*/

     force_com_PP [index]   = make_float3 (0.000000f,0.000000f,0.000000f);
     force_com_PW [index]   = make_float3 (0.000000f,0.000000f,0.000000f);

     force_ang_PP [index]   = make_float3 (0.000000f,0.000000f,0.000000f);
     force_ang_PW [index]   = make_float3 (0.000000f,0.000000f,0.000000f);

     /* If we have lifters */
     if(Num_DynamicObjects>0)
     {
        force_com_PL [index]   = make_float3 (0.000000f,0.000000f,0.000000f);
        force_ang_PL [index]   = make_float3 (0.000000f,0.000000f,0.000000f);
        Lifter_Contact_Hist[index].num_contact = 0;
        Lifter_Contact_Hist[index].cont_type = -1;
     }


     if(SimParms.use_hist)
     {
    	 Num_PPContactHist[index] =0;
     }

	 if(acc_flag==1)
	 {
	   accelrat_com [index]   = make_float3 (0.000000f,0.000000f,0.000000f);
	 }

   }

}
/*---------------------------------------------------------------------------*/


__global__ void Set_Tallys( float *Tallys )
{
	   Tally_EnergyDis_PP    = Tallys[0];
	   Tally_EnergyDisLift   = Tallys[1];
	   Tally_EnergyDisSurf   = Tallys[2];
}



/*---------------------------------------------------------------------------*/
        /* Only updates subset of particles */
/*---------------------------------------------------------------------------*/
__global__ void Fill_Mill(float pack_pos ,float3 *position_com)
{
   uint index = blockIdx.x*blockDim.x  + threadIdx.x;

   if( index <SimParms.Num_Particles )
   {

     if( position_com[index].y > pack_pos )
     {

    	 position_com[index].y = pack_pos;

     }

   }


}







/*---------------------------------------------------------------------------*/
        /* Start SIM: Sets the initial configuration values for particles */
/*---------------------------------------------------------------------------*/
__global__ void Set_Pos( float3 *Host_Pos,     Quaterion *Host_ORNT,
		                 uint   *Host_Ptype,   int   *Host_P_ID,
		                 float3 *position_com, Quaterion *position_ornt,
		                 uint   *P_ObjectType, int    *P_ID)
{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;

   if( index <SimParms.Num_Particles )
   {
	  if(SimParms.particle_type==1)
	  {
	    position_com  [index] = Host_Pos[index];// +ParticleObject[P_ObjectType[index]].COM;
	    position_ornt       [index] = Host_ORNT[index];
	  }
	  else
	  {
		position_com  [index] = Host_Pos[index];
	  }

	  if(SimParms.sphere_orient)
	  {
	   position_ornt       [index] = make_IquaterionD();
	  }


	   P_ID          [index] = Host_P_ID [index];
	   P_ObjectType  [index] = Host_Ptype[index];

   }

}


__global__ void Init_OldPos(float3 *position_com,float3 *position_comOld)
{
	   uint index = blockIdx.x*blockDim.x  + threadIdx.x;

	   if( index <SimParms.Num_Particles )
	   {
		   position_comOld[index] = position_com[index];
	   }

}



/*---------------------------------------------------------------------------*/
        /* Sets Pos and Velocity */
/*---------------------------------------------------------------------------*/
__global__ void Set_Pos_Vel( float3 *Host_Pos,     Quaterion *Host_ORNT,
		                     float3 *Host_Vel,     float3    *Host_AVel,
		                     uint   *Host_Ptype,   int       *Host_PID,
		                     float3 *position_com, Quaterion *position_ornt,
		                     float3 *velocity_com, float3    *velocity_ang,
		                     uint   *P_ObjectType,
		                     int    *P_ID)

{
   uint index = blockIdx.x*blockDim.x  + threadIdx.x;

   if( index <SimParms.Num_Particles )
   {

	  if(SimParms.particle_type==1)
	  {
		position_com  [index] = Host_Pos[index];//+ParticleObject[P_ObjectType[index]].COM;
		position_ornt [index] = Host_ORNT[index];
	  }
	  else
	  {
		position_com  [index] = Host_Pos[index];
	  }

	  velocity_com  [index] = Host_Vel  [index];
	  velocity_ang  [index] = Host_AVel [index];
	  P_ObjectType  [index] = Host_Ptype[index];
	  P_ID          [index] = Host_PID  [index];

   }

}

/*---------------------------------------------------------------------------*/
        /* Start SIM: Sets the initial configuration values for particles */
/*---------------------------------------------------------------------------*/
__global__ void Set_PSystem_State( float3 *Host_Pos,     Quaterion *Host_ORNT,
		                        float3 *Host_Vel,     float3    *Host_AVel,
		                        //float3 *Host_Acc,
		                        uint   *Host_Ptype,   int       *Host_PID,
		                        float3 *position_com, Quaterion *position_ornt,
		                        float3 *velocity_com, float3    *velocity_ang,
 		                        uint   *P_ObjectType,
 		                        int    *P_ID )

{
   uint index = blockIdx.x*blockDim.x  + threadIdx.x;

   if( index <SimParms.Num_Particles )
   {

	  if(SimParms.particle_type==1)
	  {
		position_com  [index] = Host_Pos[index] ;//+ ParticleObject[P_ObjectType[index]].COM;
		position_ornt       [index] = Host_ORNT[index];
	  }
	  else
	  {
		  position_com  [index] = Host_Pos[index];
	  }

	  if(SimParms.sphere_orient)
	  {
	   position_ornt       [index] = make_IquaterionD();
	  }



	 velocity_com  [index] = Host_Vel   [index];
	// accelrat_com  [index] = Host_Acc   [index];
	 velocity_ang  [index] = Host_AVel  [index];
	 P_ObjectType  [index] = Host_Ptype [index];
	 P_ID          [index] = Host_PID[index];

   }

}


/*---------------------------------------------------------------------------*/
           /* Returns the COM Position and Orientation to Host */
/*---------------------------------------------------------------------------*/
__global__ void Get_Particle_PosArray( float3 *Host_Pos,     Quaterion *Host_Quart,
		                               uint   *Host_PType,   int *Host_PID,
		                               float3 *position_com, Quaterion *position_ornt,
		  		                       uint   *P_ObjectType, int *P_ID, int Num_Particles                 )
{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;

	if( index <Num_Particles )
	{
	   if(SimParms.particle_type==0)
	   {
	     Host_Pos   [index] = position_com  [index];
	   }
	   else
	   {
		 Host_Pos   [index] = (position_com  [index]- ParticleObject[P_ObjectType[index]].COM);
		 Host_Quart [index] = position_ornt [index];

		 //printf(" %f %f %f \n",ParticleObject[P_ObjectType[index]].COM.x,ParticleObject[P_ObjectType[index]].COM.y,ParticleObject[P_ObjectType[index]].COM.z);
	   }

		  if(SimParms.sphere_orient)
		  {
			  Host_Quart [index] = position_ornt [index];
		  }



	   Host_PType [index] = P_ObjectType  [index];
	   Host_PID   [index] = P_ID [index];
	}

}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
           /* Returns the COM Position and Orientation to Host */
/*---------------------------------------------------------------------------*/
__global__ void Get_SystemState( float3   *Host_Pos,  Quaterion *Host_Quart,
		                         float3  *Host_Vel,   float3  *Host_AVel,
		                         uint    *Host_PType, int       *Host_PID,

		                         float3    *position_com, Quaterion *position_ornt,
		                         float3    *velocity_com, float3    *velocity_ang,
  		                         uint      *P_ObjectType, int *P_ID                )
{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;

	if( index <SimParms.Num_Particles )
	{
	   if(SimParms.particle_type==0)
	   {
	     Host_Pos   [index] = position_com  [index];
	   }
	   else
	   {
		   Host_Pos   [index] = (position_com  [index]- ParticleObject[P_ObjectType[index]].COM);
		   Host_Quart [index] = position_ornt [index];
	   }

	   Host_Vel      [index] = (velocity_com  [index]);
	   Host_AVel     [index] = (velocity_ang  [index]);
	  // Host_Acc      [index] = (accelrat_com  [index]);

	   Host_PType [index] = P_ObjectType      [index];
	   Host_PID   [index] = P_ID [index];
	}

}
/*---------------------------------------------------------------------------*/





/*---------------------------------------------------------------------------*/
           /* Returns the COM Position and Orientation to Host */
/*---------------------------------------------------------------------------*/
__global__ void Get_Particle_VelArray( float3 *Host_Vel, float3 *Host_VelR,
		                               uint    *Host_PType, int       *Host_PID,
		                               int Flag,
		                               float3 *velocity_com, float3 *velocity_ang,
		                               uint      *P_ObjectType, int *P_ID, int Num_Particles        )
{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;

	if( index < Num_Particles )
	{
	   Host_Vel      [index] = (velocity_com  [index]);

	   if (Flag==1)
	   {
	     Host_VelR     [index] = (velocity_ang  [index]);
	   }

	   Host_PType [index] = P_ObjectType [index];
	   Host_PID   [index] = P_ID         [index];
	}

}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
           /* Returns the COM Position and Orientation to Host */
/*---------------------------------------------------------------------------*/
__global__ void Get_Wall_Forces( float3 *dWall_Forces,float3 *Wall_Forces,
		                         float3 *Contact_Points )
{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;

	if( index <SimParms.Num_Particles )
	{
         Wall_Forces[index] = -1.0f*dWall_Forces[index];
	}

}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
               /* Update velocity after PP collision*/
/*---------------------------------------------------------------------------*/
__global__ void Update_VelArray(float3 *velocity_comT,float3 *velocity_com)
{
  uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

  if( index <SimParms.Num_Particles )
  {
    velocity_com[index] = velocity_comT[index];
  }

}



__global__ void ResetEnergy(int num)
{

	if(num<=3)
	{
	  Tally_EnergyDis_PP  = 0.0f;
	  Tally_EnergyDisLift = 0.0f;
	  Tally_EnergyDisSurf = 0.0f;
	}

	if(num>3)
	{
	  Tally_EnergyDis_Norm  = 0.0f;
	  Tally_EnergyDis_Shear = 0.0f;
	}

}

__global__ void GetEnergy(float *H_Energy, int num)
{
	if(num<=3)
    {
	  H_Energy[0]=Tally_EnergyDis_PP*1E-4;
	  H_Energy[1]=Tally_EnergyDisLift*1E-4;
	  H_Energy[2]=Tally_EnergyDisSurf*1E-4;
    }

	if(num>3)
	{
	 H_Energy[3]=Tally_EnergyDis_Norm*1E-4;
	 H_Energy[4]=Tally_EnergyDis_Shear*1E-4;
	}

}



/* Flags particles which meet the criteria below */
__global__ void Silo_Kill(float3 *position_com, int *P_ID)
{
    uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

	if( index < SimParms.Num_Particles  && P_ID[index]>-1)
	{
		if(position_com[index].y<SimParms.Silo_Kill_Height)
		{
			 position_com[index]=SimParms.max_size;
			 P_ID[index]=-1;
			 atomicAdd(&Kill_count,1);
		}

	}
}


__global__ void Kill_copy(int *killed)
{

    killed[0]=Kill_count;
}



/* Flags particles which meet the criteria below */
__global__ void Remove_Silo_Discharged_Particles(float3  *position_com, int *P_ID,int Num_Particles )
{
    uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

	if( (index < Num_Particles) && P_ID[index]!=-1 )
	{
		if( position_com[index].y < SimParms.Silo_Kill_Height )
		{
			 position_com[index]=SimParms.max_size;
			 P_ID[index]=-1;
			 atomicAdd(&Kill_count,1);
		}

	}
}


/* Flags particles which meet the criteria below */
__global__ void Remove_PL_Discharged_Particles(float3 *position_com, int *P_ID,int Num_Particles )
{
    uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

	if( (index < Num_Particles) && P_ID[index]!=-1 )
	{
		if( position_com[index].z <= 41.0f ||position_com[index].z >= 300  || position_com[index].y <= 50.0f || position_com[index].y >= 1500.0f|| position_com[index].x <= 50.0f || position_com[index].x >= 1500.0f)
		{
			 position_com[index] = SimParms.max_size;
			 P_ID[index]=-1;
			 atomicAdd(&Kill_count,1);
		}

	}
}





__global__ void AddForce()
{
	fc+=10.0f;
}

__global__ void SubForce()
{
	fc-=10.0f;
}

__global__ void ZeroForce()
{
	fc=0.0f;
}

__global__ void Hatch()
{
	Num_WorldObjects = Num_WorldObjects -1;
}


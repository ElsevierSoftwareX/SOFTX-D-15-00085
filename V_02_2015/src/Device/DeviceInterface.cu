
/* Change List
 * 1) 23/12/2012 -- adding OpenGL VBO*/

/*---------------------------------------------------------------------------*/
                    /* REQUIRED INCLUDE FILES*/
/*---------------------------------------------------------------------------*/

                       /* System C++ */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <iostream>
#include <fstream>

                       /* System CUDA */
#include <cuda_runtime.h>
#include <thrust/for_each.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/sort.h>
#include <thrust/remove.h>
#include <thrust/scan.h>
#include <thrust/count.h>
#include <thrust/sequence.h>
#include <thrust/device_vector.h>
/*------------------------------------------------*/

                       /* Data Structures  */
#include "../DataStructures/KSimulationObjects.h"

                       /* Public Methods  */
#include "DeviceInterface.h"

/*---------------------------------------------------------------------------*/
/*                          Device includes                                  */
/*---------------------------------------------------------------------------*/
#include "../Utilities/Device/Opertators.cuh"
#include "../Utilities/Device/Functions.cuh"


#include "GPU_Memory_Transfer.cuh"

#include <string>
#include <sstream>


int error_check=0;


/* Particle Dynamics Information Global memory  */

/* Translational Parameters */
float3     *dram_position_com;
float3    *dram_velocity_com;

     /* PP Forces */
float *dram_force_com_X;
float *dram_force_com_Y;
float *dram_force_com_Z;

/* non-symmetry */
float3 *force_PP;


float3 *dram_force_com_Wall;
float3 *dram_force_com_Lifter;

/* Angular Parameters */
Quaterion *dram_position_ornt;
float3    *dram_velocity_ang;

   /* PP Forces */
float *dram_force_ang_X;
float *dram_force_ang_Y;
float *dram_force_ang_Z;

/* non-symmetry */
float3 *force_PP_ang;

float3 *dram_force_ang_Lifter;
float3 *dram_force_ang_Wall;

/* Particle Geometry identifiers */
uint   *dram_ObjectType;
int    *dram_P_ID;

/*----------------------------------------------------------------------*/

float3   *dram_accelrat_com;    /* Optional: For velocity  Verlet */
float3   *dram_position_com_old;/* Optional: For classic   Verlet */


/* Contact point info for post processing */
float3 *dWallContactPoints;  /* Optional */
float3 *dLifterContactPoints;/* Optional */

/* Spatial Grid parameters */
uint  *m_dCellStart;
uint  *m_dCellEnd;
int    m_numGridCells;

uint  *dram_GridParticleHash; /* Grid hash value for each particle */
int   *dram_GridParticleIndex;/* Particle index for each particle used to sort */

/* History contact for particle Lifter/DObject */
Contact_Info *dram_Lifter_Contact_Hist;


Contact_InfoPP *dram_PP_Contact_Hist;
int            *dram_PP_Contact_Num;


#include "Utilities.cuh"
/*                        DEM Kernels for Polyhedra                          */
#include "Computing/Collision_Detection_BroadPhase.cuh"
/*                        DEM Kernels for Spheres                            */
#include "Computing/DEM_Kernels_Compute.cuh"
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
                         /* Local Variables */
/*---------------------------------------------------------------------------*/

/* Flags */
ofstream OF;
bool Device_use_multi_gpu = false;
int  run_level = 0;
int  num_sm;
int  num_contacts=0;
bool start_bin            = false;
bool is_cylinder_rotation = false;
bool Get_WallPoints       = false;

int            num_devices;
cudaDeviceProp DevProp;
size_t t;
size_t a;



/* Local Information */
int             NUMWOBJECTS;
int             NUMPARTICLES_MAX_SIM;
int             NUMDYOBJECTS;
launchParms     GlobalLaunch;
SimulationInfo *SimInfo_C;



/* Data used for iniial filling */
float3          *h_PosPack;             /* Initial Pos */
Quaterion       *h_OrntPack;             /* Initial Pos */
uint            *h_Particle_TypePack; /* Initial Particle TypeID */
int             *h_Particle_IDPack; /* Initial Particle ID */

ofstream        LogFileD;
ofstream        EnergyBin;
int             device_num = 0;

int num_dead = 0;
int stepc    = 0;

int D_NUMPARTICLES;
int NUMPARTICLES_Current;


bool   isFilling= false;
float3 pack_pos;
int3   pack_size;

int    packNumber;
int    packNumWaves;
int    threads_perBlock;
int    fill_counter=0;
int    pack_clear_steps=-1;

bool is_first=true;


int NumPhysicsTallies = 3;

float Call_Time=0.0f;




/*---------------------------------------------------------------------------*/
          /* Set the Initial distribution as a AxBxC grid */
/*---------------------------------------------------------------------------*/
void Set_Position_DefaultGrid( InitConfig *h_PosConfig, POBJ *h_POBJ )
{

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Initial Position Default Grid: Start "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

	cout<<"INFO-D: Allocating Initial positions on HOST \n";

	h_PosPack             = new float3    [NUMPARTICLES_MAX_SIM];

	 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	 {
	   h_OrntPack            = new Quaterion [NUMPARTICLES_MAX_SIM];
	 }

	h_Particle_TypePack   = new uint      [NUMPARTICLES_MAX_SIM];
	h_Particle_IDPack     = new int       [NUMPARTICLES_MAX_SIM];


	cout<<"INFO-D: Creating Fixed Position Grid  "<<NUMPARTICLES_MAX_SIM<<endl;


	cout<<" \n";

    float x,y,z;
    float x_size,y_size,z_size;


    int    NX  = h_PosConfig->num.x;
    int    NY  = h_PosConfig->num.y;
    int    NZ  = h_PosConfig->num.z;

    x_size = h_PosConfig->p_size[0].x;
    y_size = h_PosConfig->p_size[0].y;
    z_size = h_PosConfig->p_size[0].z;


    x      = h_PosConfig->start.x;
    y      = h_PosConfig->start.y;
    z      = h_PosConfig->start.z;



    cout<<"INFO-D: Start  "<<x<<", "<<y<<", "<<z;
    cout<<"  Grid  "<<NX<<", "<<NY<<", "<<NZ;

    /* Allocate positions */
    for ( int yd=0; yd< NY; yd++ )/* Height  */
      {

         for ( int zd=0; zd<NZ; zd++ )/* x B-F */
         {
      	  for ( int xd=0; xd< NX; xd++ ) /* L-R */
      	  {

      	    h_PosPack [ yd*NZ*NX + zd*NX   + xd ] = make_float3(x,y,z);

      		 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
      		 {
      	       h_OrntPack[ yd*NZ*NX + zd*NX   + xd ] = make_quaterion(0,0.0,0.0,0.0);
      		 }

              x = x + x_size + h_PosConfig->space.x;
           }


      	    x = h_PosConfig->start.x;
            z = z + z_size + h_PosConfig->space.z;
          }

          z = h_PosConfig->start.z;
          y = y + y_size + h_PosConfig->space.y; /* next height level*/
       }



    cout<<"INFO-D: Allocating Initial Positions on Device: ";



	float3 *d_posPack;
	cudaMalloc( (void**) &d_posPack , sizeof(float3)*NUMPARTICLES_MAX_SIM );
	cudaDeviceSynchronize();

	cout<<" DONE! \n";

	cout<<"INFO-D: Copying  Host Positions to Device: ";
    cudaMemcpy(d_posPack, h_PosPack, sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    cudaDeviceSynchronize();
    cout<<" DONE! \n";

	/* Set the particle type */
	int index_particle = 0;

	/* Now store particle types */
	for( int i=0; i<SimInfo_C->Num_ParticleObjects; i++ )
	{
		for(int j=0; j<SimInfo_C->Num_ParticlesPerObject[i]; j++)
		{
			h_Particle_TypePack[index_particle] = i;
			h_Particle_IDPack  [index_particle] = index_particle;
			index_particle++;
		}

	 }


	 LogFileD<<"\n";

	 /* Copy particle type to device */
	 uint *d_Particle_TypePack;
	 int  *d_Particle_IDPack;

	 LogFileD<<"INFO-D: Allocating Particle Types on Device: ";

	 cudaMalloc( (void**) &d_Particle_TypePack , sizeof(uint)*NUMPARTICLES_MAX_SIM );
	 cudaMalloc( (void**) &d_Particle_IDPack   , sizeof(int)*NUMPARTICLES_MAX_SIM );
	 cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	        {
	      	 cout<<"Initial Position Default Grid: Malloc ID "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }




	 LogFileD<<" DONE! \n";
	 LogFileD<<"INFO-D: Copying  Particle Types to Device:";

	 cudaMemcpy( d_Particle_TypePack, h_Particle_TypePack, sizeof(uint)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	 cudaMemcpy( d_Particle_IDPack,   h_Particle_IDPack, sizeof(int)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	 cudaDeviceSynchronize();
	 LogFileD<<" DONE! \n";
	 LogFileD<<"\n";


	 LogFileD<<"INFO-D: Initializing arrays on Device: ";



	 Quaterion *d_orntPack;
	 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	 {
			cudaMalloc( (void**) &d_orntPack , sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
			cudaDeviceSynchronize();
		    cudaMemcpy(d_orntPack, h_OrntPack, sizeof(Quaterion)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
		    cudaDeviceSynchronize();

	 }


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	        {
	      	 cout<<"Initial Position Default Grid: Memory End "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }



	 Set_Pos<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>( d_posPack,
			                                                     d_orntPack,
                                                                 d_Particle_TypePack,
                                                                 d_Particle_IDPack,
                                                                 dram_position_com,
			                                                     dram_position_ornt,
			                                                     dram_ObjectType,
			                                                     dram_P_ID            );

	 cudaDeviceSynchronize();


	 if(error_check==1)
	 {
	    cudaError_t errormsg=cudaGetLastError();
	    if(errormsg>0)
	    {
	       cout<<"Initial Position Default Grid: Set Pos Kernel "<<cudaGetErrorString(errormsg)<<endl;
	       exit(1);
	    }
	 }




	 LogFileD<<" DONE Freeing Memory ! \n";

     delete [] h_PosPack;

	 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	 {
       delete [] h_OrntPack;
	 }

     delete [] h_Particle_TypePack;
     delete [] h_Particle_IDPack;

	 cudaFree( d_posPack );
     if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
	 {
	   cudaFree( d_orntPack );
	 }
	 cudaFree( d_Particle_TypePack );
	 cudaFree( d_Particle_IDPack );

	 cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	        {
	    	  cout<<"Initial Position Default Grid: Free memory "<<cudaGetErrorString(errormsg)<<endl;
	    	  exit(1);
	    	}
	 }

}
/*-----------------------------------------------------------------------------*/





/*---------------------------------------------------------------------------*/
          /* Set Grid Random Distribution of type and *Orientation  */
/*---------------------------------------------------------------------------*/
void Set_Position_Random_Grid ( InitConfig *h_PosConfig, POBJ *h_POBJ )
{

	cout<<"INFO-D: Allocating Initial Random positions on HOST \n";
	h_PosPack           = new float3    [NUMPARTICLES_MAX_SIM];

    if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
	  h_OrntPack          = new Quaterion [NUMPARTICLES_MAX_SIM];
	}

	h_Particle_TypePack = new uint      [NUMPARTICLES_MAX_SIM];
	h_Particle_IDPack   = new int      [NUMPARTICLES_MAX_SIM];


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Initial Position Random Grid: Start "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	cout<<"INFO-D: Creating Random Grid \n";

	cout<<" \n";

    float x,y,z;
    float x_size,y_size,z_size;


    int    NX  = h_PosConfig->num.x;
    int    NY  = h_PosConfig->num.y;
    int    NZ  = h_PosConfig->num.z;

    x_size = h_PosConfig->p_size[0].x;
    y_size = h_PosConfig->p_size[0].y;
    z_size = h_PosConfig->p_size[0].z;


    x      = h_PosConfig->start.x;
    y      = h_PosConfig->start.y;
    z      = h_PosConfig->start.z;

    int selectcount[6];

    /* Get prob for each type */
 	for(int i=0; i<SimInfo_C->Num_ParticleObjects; i++)
 	{
 	  selectcount[i]=0;
 	}


 	int selectT=-1;
 	int pty=0;


    for ( int yd=0; yd< NY; yd++ )/* Height  */
    {

       for ( int zd=0; zd<NZ; zd++ )/* x B-F */
       {
    	  for ( int xd=0; xd< NX; xd++ ) /* L-R */
    	  {

    	    if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
    	    {
				h_PosPack[ yd*NZ*NX + zd*NX   + xd ]= make_float3(x,y,z);

				/* Random Orientation */
				float rand_w=((float)rand()/RAND_MAX)*30.0;
				float rand_x=(float)rand()/RAND_MAX;
				float rand_y=(float)rand()/RAND_MAX;
				float rand_z=(float)rand()/RAND_MAX;
				h_OrntPack[ yd*NZ*NX + zd*NX   + xd ]= make_quaterion(rand_w,rand_x,rand_y,rand_z);//(90.0,0.0,1.0,0.0);//
    	    }

      	    selectT++;

    	      /* Reset after each alternating */
    	      if(selectT >= SimInfo_C->Num_ParticleObjects )
    	      {
    	    	  selectT=0;
    	    	  //printf("reset\n");
    	    	  //printf("\n");
    	      }

    	      pty=selectT;


              /* Make sure we dont exceed count */
    	      if( selectcount[pty]>=SimInfo_C->Num_ParticlesPerObject[pty] )
    	      {
    	       /* Find someone who is free */
    	    	 	for(int i=0; i<SimInfo_C->Num_ParticleObjects; i++)
    	    	 	{
    	      	      if( selectcount[i]<=(SimInfo_C->Num_ParticlesPerObject[i]-1) )
    	      	      {
    	      	    	//printf("  swap %d %d count %d num %d  ",selectT,i,selectcount[i],(SimInfo_C->Num_ParticlesType[i]-1));
    	      	    	pty=i;
    	      	    	break;
    	      	      }
    	    	 	}
    	      }

    	     selectcount[pty]++;


    	    h_Particle_TypePack[yd*NZ*NX + zd*NX   + xd] = pty;

    	    h_Particle_IDPack  [yd*NZ*NX + zd*NX   + xd] = yd*NZ*NX + zd*NX   + xd;

            x = x + x_size + h_PosConfig->space.x;
          }


    	  x = h_PosConfig->start.x;
          z = z + z_size + h_PosConfig->space.z;
        }

        z = h_PosConfig->start.z;
        y = y + y_size + h_PosConfig->space.y; /* next height level*/
     }


     /* Now randomize grid */

		int source;
		int dest;

		int temp;

		for(int i=0; i<NUMPARTICLES_MAX_SIM;i ++)
		{
			source = rand()%(NUMPARTICLES_MAX_SIM-1);
			dest   = rand()%(NUMPARTICLES_MAX_SIM-1);

			/* Sort P_Type */
			temp                        = h_Particle_TypePack[source];
			h_Particle_TypePack[source] = h_Particle_TypePack[dest];
			h_Particle_TypePack[dest]   = temp;

			temp                        = h_Particle_IDPack[source];
			h_Particle_IDPack[source] = h_Particle_IDPack[dest];
			h_Particle_IDPack[dest]   = temp;

		}



    cout<<"INFO-D: Allocating Initial Positions on Device: ";

	float3 *d_posPack;
	cudaMalloc( (void**) &d_posPack , sizeof(float3)*NUMPARTICLES_MAX_SIM );
	cudaDeviceSynchronize();


	cout<<" DONE! \n";

	cout<<"INFO-D: Copying  Positions to Device: ";
    cudaMemcpy(d_posPack, h_PosPack, sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    cudaDeviceSynchronize();


    cout<<" DONE! \n";


	LogFileD<<"\n";
	 /* Copy particle type to device */
	uint *d_Particle_TypePack;
	int *d_Particle_IDPack;

	 LogFileD<<"INFO-D: Allocating Particle Types on Device: ";

	 cudaMalloc( (void**) &d_Particle_TypePack , sizeof(uint)*NUMPARTICLES_MAX_SIM );
	 cudaMalloc( (void**) &d_Particle_IDPack , sizeof(uint)*NUMPARTICLES_MAX_SIM );
	 cudaDeviceSynchronize();
	 LogFileD<<" DONE! \n";
	 LogFileD<<"INFO-D: Copying  Particle Types to Device:";

	 cudaMemcpy( d_Particle_TypePack, h_Particle_TypePack, sizeof(uint)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	 cudaMemcpy( d_Particle_IDPack, h_Particle_IDPack, sizeof(uint)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	 cudaDeviceSynchronize();

	 Quaterion *d_orntPack;

	 if( SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient )
	 {

			cudaMalloc( (void**) &d_orntPack , sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
			cudaDeviceSynchronize();
		    cudaMemcpy(d_orntPack, h_OrntPack, sizeof(Quaterion)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
		    cudaDeviceSynchronize();

	 }

	 LogFileD<<" DONE! \n";
	 LogFileD<<"\n";


	 LogFileD<<"INFO-D: Initializing arrays on Device: ";

	 cudaDeviceSynchronize();


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Initial Position Random Grid: Mem end "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	 Set_Pos<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>( d_posPack,
			                                                          d_orntPack,
                                                                      d_Particle_TypePack,
                                                                      d_Particle_IDPack,
                                                                      dram_position_com,
			                                                          dram_position_ornt,
			                                                          dram_ObjectType,dram_P_ID);
	 cudaDeviceSynchronize();


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Initial Position Random Grid: Set_Pos Kernel "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

     cout<<" DONE Freeing Memory ! \n";

     delete [] h_PosPack;

	 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	 {
     delete [] h_OrntPack;
	 }
     delete [] h_Particle_TypePack;
     delete [] h_Particle_IDPack;

	 cudaFree( d_posPack );
	 if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
	 {
	  cudaFree( d_orntPack );
	 }
	 cudaFree( d_Particle_TypePack);
	 cudaFree( d_Particle_IDPack);

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	 cout<<"Initial Position Random Grid: Mem Free "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }
}
/*-----------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
          /* Fill from half-way up   */
/*---------------------------------------------------------------------------*/
void Set_Position_Fill_Mill (POBJ *h_POBJ,WOBJ *h_WOBJ, float vel, float3 fill_planeStart, float3 fill_planeEnd )
{

	cout<<"INFO-D: Allocating Fill positions on HOST \n";

	h_PosPack             = new float3    [NUMPARTICLES_MAX_SIM];

	if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
	  h_OrntPack            = new Quaterion [NUMPARTICLES_MAX_SIM];
	}

	h_Particle_TypePack   = new uint      [NUMPARTICLES_MAX_SIM];
	h_Particle_IDPack     = new int       [NUMPARTICLES_MAX_SIM];

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	  cout<<"Initial Position Fill: Start "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

	cout<<"INFO-D: Creating Fill Config \n";

	cout<<" \n";

    float x,y,z;
    float x_size,y_size,z_size;

    float3 space;
    float3 start;
    space = make_float3(0.00100f,0.0010f,0.0000f);

    /* Biggest particle is first */
    x_size = 2.0*h_POBJ[0].radius;
    y_size = 2.0*h_POBJ[0].radius;
    z_size = 2.0*h_POBJ[0].radius;

    /* For mill box width is diameter */
    int    NX,NY,NZ;

    float radius;
    if(NUMDYOBJECTS>0)
    {
     radius = SimInfo_C->DObject_CylinderBoundR;
    }
    else
    {
    	radius = h_WOBJ[0].cyl_geo.radius;
    }


    if(SimInfo_C->Simulation_Type==ballmill && !SimInfo_C->Rotating_WObject)
    {
      printf("Mill detected\n");

      if(NUMDYOBJECTS>0)
      {
        x      =  x_size + (h_WOBJ[0].cyl_geo.radius - SimInfo_C->DObject_CylinderBoundR);
      }
      else
      {
    	  x    =  x_size + space.x;
      }

      y      = h_WOBJ[0].cyl_geo.radius;

      z      =  h_WOBJ[0].cyl_geo.center_bot_cap.z
    		    + z_size/2.0f + space.z;

      NX  = floor( ( radius*2 - x_size )/(x_size + space.x) );

      NZ  = floor((h_WOBJ[0].cyl_geo.height)/(z_size + space.z));

      NY  = ceil( NUMPARTICLES_MAX_SIM/(float)(NX*NZ) );

    }
    else
    {

    	 printf("Planes Detected \n");
      x      = fill_planeStart.x
    		    + x_size;

      y      = fill_planeStart.y;

      z      = fill_planeStart.z
    		     + z_size;

      NX  = floor((fill_planeEnd.x - fill_planeStart.x)/(x_size + space.x));
      NZ  = floor((fill_planeEnd.z - fill_planeStart.z) /(z_size + space.z));

      NY  = ceilf( NUMPARTICLES_MAX_SIM/(float)(NX*NZ) );

    }

    start = make_float3(x,y,z);

    pack_pos = start;


    printf("Start %f %f %f  \n",x,y,z);

    printf("Grid Size %d %d %d Tot %d \n",NX,NY,NZ, NX*NY*NZ);

    pack_size = make_int3(NX,NY,NZ);

    packNumber = NX*NZ;

    packNumWaves= NY;



    pack_clear_steps = fabs((int)ceil((2.0f*y_size/(SimInfo_C->InitalDelta_t*vel))));
    printf("clear steps %d \n",pack_clear_steps);

    int selectcount[6];

    /* Get prob for each type */
 	for(int i=0; i<SimInfo_C->Num_ParticleObjects; i++)
 	{
 	  selectcount[i] = 0;
 	}


 	int selectT=-1;
 	int pty=0;


    for ( int yd=0; yd< NY; yd++ )/* Height  */
    {

       for ( int zd=0; zd<NZ; zd++ )/* x B-F */
       {
    	  for ( int xd=0; xd< NX; xd++ ) /* L-R */
    	  {
    		if(yd*NZ*NX + zd*NX   + xd <NUMPARTICLES_MAX_SIM)
    		{
    	      h_PosPack[ yd*NZ*NX + zd*NX   + xd ]= make_float3(x,y,z);


    	 	 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
    	 	 {	      /* Random Orientation */
      	      float rand_w=((float)rand()/RAND_MAX)*30.0;
      	      float rand_x=(float)rand()/RAND_MAX;
      	      float rand_y=(float)rand()/RAND_MAX;
      	      float rand_z=(float)rand()/RAND_MAX;
      	      h_OrntPack[ yd*NZ*NX + zd*NX   + xd ]= make_quaterion(rand_w,rand_x,rand_y,rand_z);//(90.0,0.0,1.0,0.0);//
    	    }

      	    selectT++;

    	      /* Reset after each alternating */
    	      if(selectT >= SimInfo_C->Num_ParticleObjects )
    	      {
    	    	  selectT=0;
   	           }

    	      pty=selectT;


              /* Make sure we dont exceed count */
    	      if( selectcount[pty]>=SimInfo_C->Num_ParticlesPerObject[pty] )
    	      {
    	       /* Find someone who is free */
    	    	 	for(int i=0; i<SimInfo_C->Num_ParticleObjects; i++)
    	    	 	{
    	      	      if( selectcount[i]<=(SimInfo_C->Num_ParticlesPerObject[i]-1) )
    	      	      {
    	      	    	pty=i;
    	      	    	break;
    	      	      }
    	    	 	}
    	      }

    	    selectcount[pty]++;


    	    h_Particle_TypePack[yd*NZ*NX + zd*NX   + xd] = pty;
    	    h_Particle_IDPack[yd*NZ*NX + zd*NX   + xd] = yd*NZ*NX + zd*NX   + xd;

            x = x + x_size + space.x;
          }
    	  else
    	  {
    	     break;
    	   }
       }


    	  x = start.x;
          z = z + z_size + space.z;
        }

        z = start.z;

       y = y + y_size + space.y; /* next height level*/
     }



    uint  *d_Particle_TypePack;
    int   *d_Particle_IDPack;
     /* Now randomize grid */

		int source;
		int dest;

		int temp;

		for(int i=0; i<NUMPARTICLES_MAX_SIM;i ++)
		{
			source = rand()%(NUMPARTICLES_MAX_SIM-1);
			dest   = rand()%(NUMPARTICLES_MAX_SIM-1);

			temp                        = h_Particle_TypePack[source];
			h_Particle_TypePack[source] = h_Particle_TypePack[dest];
			h_Particle_TypePack[dest]   = temp;

			temp                        = h_Particle_IDPack[source];
			h_Particle_IDPack[source]   = h_Particle_IDPack[dest];
			h_Particle_IDPack[dest]     = temp;
		}



    cout<<"INFO-D: Allocating Initial Positions on Device: ";

    float3 *d_posPack;
	cudaMalloc( (void**) &d_posPack , sizeof(float3)*NUMPARTICLES_MAX_SIM );
	cudaDeviceSynchronize();


	cout<<" DONE! \n";


	cout<<"INFO-D: Copying  Positions to Device: ";
    cudaMemcpy(d_posPack, h_PosPack, sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    cudaDeviceSynchronize();
    cout<<" DONE! \n";


	LogFileD<<"\n";
	 /* Copy particle type to device */


	 LogFileD<<"INFO-D: Allocating Particle Types on Device: ";

	 cudaMalloc( (void**) &d_Particle_TypePack , sizeof(uint)*NUMPARTICLES_MAX_SIM );
	 cudaMalloc( (void**) &d_Particle_IDPack , sizeof(int)*NUMPARTICLES_MAX_SIM );
	 cudaDeviceSynchronize();
	 LogFileD<<" DONE! \n";
	 LogFileD<<"INFO-D: Copying  Particle Types to Device:";

	 cudaMemcpy( d_Particle_TypePack, h_Particle_TypePack, sizeof(uint)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	 cudaMemcpy( d_Particle_IDPack,   h_Particle_IDPack, sizeof(int)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	 cudaDeviceSynchronize();

	 Quaterion *d_orntPack;
	 if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
	 {

			cudaMalloc( (void**) &d_orntPack , sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
			cudaDeviceSynchronize();
		    cudaMemcpy(d_orntPack, h_OrntPack, sizeof(Quaterion)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
		    cudaDeviceSynchronize();

	 }

	 LogFileD<<" DONE! \n";
	 LogFileD<<"\n";


	 LogFileD<<"INFO-D: Initializing arrays on Device: ";
	 cudaDeviceSynchronize();


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Initial Position Fill: Mem End "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	 Set_Pos<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>( d_posPack,
			                                                     d_orntPack,
                                                                 d_Particle_TypePack,
                                                                 d_Particle_IDPack,
                                                                 dram_position_com,
			                                                     dram_position_ornt,
			                                                     dram_ObjectType,
			                                                     dram_P_ID           );
	 cudaDeviceSynchronize();


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Initial Position Fill: Set_Pos Kernel "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

     cout<<" DONE Freeing Memory ! \n";

     NUMPARTICLES_Current = 0;

     delete [] h_PosPack;

	 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	 {
       delete [] h_OrntPack;
	 }
     delete [] h_Particle_TypePack;
     delete [] h_Particle_IDPack;

	 cudaFree( d_posPack );
	 if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	 {
	  cudaFree( d_orntPack );
	 }

	 cudaFree( d_Particle_TypePack );
	 cudaFree( d_Particle_IDPack );

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Initial Position Fill: Mem Free "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }
}
/*-----------------------------------------------------------------------------*/

/* Adds particles to the simulation until NUMPARTICLES_Max is reached */
void Fill_State()
{
	int num_rem      = NUMPARTICLES_MAX_SIM - NUMPARTICLES_Current;
	int num_particle = min(pack_size.x*pack_size.z,num_rem);


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<" Fill State: Start "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }



	NUMPARTICLES_Current += num_particle;


	   int num_particles_SM = (int)ceil(NUMPARTICLES_Current/(float)num_sm);

	   /* Now get the number of blocks per SM */
	   int num_blocks_SM =  (int)ceil(num_particles_SM/(float)threads_perBlock);

	   int num_threads_block=threads_perBlock;

	   /* block size is too big */
	   if(num_particles_SM < threads_perBlock)
	   {
		   num_threads_block = num_particles_SM; /* Single block is sufficient */
	   }

	   GlobalLaunch.dimBlock = make_uint3( num_threads_block,1,1);
	   GlobalLaunch.dimGrid  = make_uint3( num_blocks_SM*num_sm,1,1 );



	    SimInfo_C->Num_Particles = NUMPARTICLES_Current;
	    cudaMemcpyToSymbol( SimParms,       SimInfo_C,     sizeof( SimulationInfo ) );
	    cudaDeviceSynchronize();

	/* Only modify subset */
    Fill_Mill<<<GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>(pack_pos.y,dram_position_com);
	cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Fill State: Fill_Mill Kernel "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

}



void Device_Set_Pos( float3 *h_pos,  Quaterion *h_Ornt,
		             uint *h_ptype,   int       *h_pid   )
{

    LogFileD<<"INFO-D: Allocating File Positions on Device: ";

	float3    *d_pos;
	Quaterion *d_Ornt;
	uint      *d_ptype;
	int       *d_pid;


	cudaMalloc( (void**) &d_pos , sizeof(float3)*NUMPARTICLES_MAX_SIM );
	cudaMalloc( (void**) &d_ptype, sizeof(uint) *NUMPARTICLES_MAX_SIM  );
	cudaMalloc( (void**) &d_pid,   sizeof(int)*NUMPARTICLES_MAX_SIM    );
	cudaDeviceSynchronize();


	LogFileD<<" DONE! \n";
	LogFileD<<"INFO-D: Copying  Positions to Device: ";

	cudaMemcpy(d_pos, h_pos, sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );

	if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
	{
		cudaMalloc( (void**) &d_Ornt , sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
		cudaDeviceSynchronize();
		cudaMemcpy(d_Ornt, h_Ornt, sizeof(Quaterion)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	}

    cudaMemcpy(d_ptype, h_ptype, sizeof(uint)*NUMPARTICLES_MAX_SIM,  cudaMemcpyHostToDevice );
    cudaMemcpy(d_pid,   h_pid,  sizeof(int)*NUMPARTICLES_MAX_SIM,  cudaMemcpyHostToDevice );
    cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set Position: Mem  End "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }



    LogFileD<<" DONE! \n";
    Set_Pos<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>( d_pos,d_Ornt, d_ptype, d_pid,
    		                                                    dram_position_com, dram_position_ornt,
    		                                                     dram_ObjectType, dram_P_ID   );
	cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set Position: Set_Pos Kernel "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	cudaFree(d_pos);
	if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
	  cudaFree(d_Ornt);
	}

	cudaFree(d_ptype);
	cudaFree(d_pid);

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set Position: Mem Free "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	LogFileD<<" DONE Freeing Memory ! \n";

}
/*-----------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
                 /* Set SPEC TYPE:2  Pos and Velocity */
/*---------------------------------------------------------------------------*/
void Device_Set_Pos_Velocity( float3 *h_pos, Quaterion *h_Ornt, float3 *h_vel,
		                      float3 *h_Avel, uint *h_ptype,  int     *h_pid)
{

    LogFileD<<"INFO-D: Allocating File Positions on Device: ";


	if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set Position Velocity: Start "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	float3    *d_pos;
	float3    *d_vel;
	float3    *d_Avel;
	Quaterion *d_Ornt;
	uint      *d_ptype;
	int       *d_pid;


	cudaMalloc( (void**) &d_pos , sizeof(float3)*NUMPARTICLES_MAX_SIM );

	if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
	{
		cudaMalloc( (void**) &d_Ornt , sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
	}

	cudaMalloc( (void**) &d_vel , sizeof(float3)*NUMPARTICLES_MAX_SIM );
	cudaMalloc( (void**) &d_Avel, sizeof(float3)*NUMPARTICLES_MAX_SIM );
	cudaMalloc( (void**) &d_ptype, sizeof(uint) *NUMPARTICLES_MAX_SIM  );
	cudaMalloc( (void**) &d_pid,   sizeof(int)*NUMPARTICLES_MAX_SIM    );
	cudaDeviceSynchronize();


	LogFileD<<" DONE! \n";
	LogFileD<<"INFO-D: Copying  Positions to Device: ";

	cudaMemcpy(d_pos, h_pos, sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );

	if(SimInfo_C->particle_type==1)
	{
		cudaMemcpy(d_Ornt, h_Ornt, sizeof(Quaterion)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
	}

    cudaMemcpy(d_vel,  h_vel,   sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    cudaMemcpy(d_Avel, h_Avel,  sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    cudaMemcpy(d_ptype, h_ptype, sizeof(uint)*NUMPARTICLES_MAX_SIM,  cudaMemcpyHostToDevice );
    cudaMemcpy(d_pid,   h_pid,   sizeof(int)*NUMPARTICLES_MAX_SIM,  cudaMemcpyHostToDevice );
    cudaDeviceSynchronize();

    LogFileD<<" DONE! \n";
    Set_Pos_Vel<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>( d_pos,d_Ornt,d_vel,d_Avel,
    		                                                        d_ptype,d_pid,
    		                                                        dram_position_com, dram_position_ornt,
    		                                                         dram_velocity_com, dram_velocity_ang, dram_ObjectType, dram_P_ID   );
	cudaDeviceSynchronize();

	if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set Position Velocity: Set_Pos_Vel Kernel "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }



	cudaFree(d_pos);
	cudaFree(d_vel);
	cudaFree(d_Avel);

	if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
	{
	  cudaFree(d_Ornt);
	}

	cudaFree(d_ptype);
	cudaFree(d_pid);

	LogFileD<<" DONE Freeing Memory ! \n";

}
/*-----------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
          /* Spec type 3 all particle properties */
/*---------------------------------------------------------------------------*/
void Device_Set_System_State( float3    *h_pos,
		                      Quaterion *h_Ornt,
		                      float3 *h_vel,
		                      float3 *h_Avel,
		                      float3 *h_acc,
		                      uint  *h_ptype,
		                      int      *h_pid)
{

    cout<<"INFO-D: Allocating File Positions on Device: ";

	float3    *d_pos;
	float3    *d_vel;
//	float3    *d_acc;
	Quaterion *d_Ornt;
	float3    *d_Avel;
	uint      *d_ptype;
	int       *d_pid;

	cudaMalloc( (void**) &d_pos  , sizeof(float3)   *NUMPARTICLES_MAX_SIM );

	if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
    {
      cudaMalloc( (void**) &d_Ornt , sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
    }

	cudaMalloc( (void**) &d_vel  , sizeof(float3)   *NUMPARTICLES_MAX_SIM );
	cudaMalloc( (void**) &d_Avel , sizeof(float3)   *NUMPARTICLES_MAX_SIM );
	//cudaMalloc( (void**) &d_acc  , sizeof(float3)   *NUMPARTICLES_MAX_SIM );
	cudaMalloc( (void**) &d_ptype , sizeof(uint)     *NUMPARTICLES_MAX_SIM );
	cudaMalloc( (void**) &d_pid   , sizeof(int)     *NUMPARTICLES_MAX_SIM );
	cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set System State: Start "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	cout<<" DONE! \n";

	cout<<"INFO-D: Copying  State to Device: ";

	cudaMemcpy( d_pos,  h_pos,  sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );

	if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
    {
      cudaMemcpy(d_Ornt, h_Ornt, sizeof(Quaterion)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    }
    cudaMemcpy( d_vel,  h_vel,  sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    //cudaMemcpy( d_acc,  h_acc,  sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    cudaMemcpy( d_Avel, h_Avel, sizeof(float3)*NUMPARTICLES_MAX_SIM, cudaMemcpyHostToDevice );
    cudaMemcpy( d_ptype, h_ptype, sizeof(uint)*NUMPARTICLES_MAX_SIM,   cudaMemcpyHostToDevice );
    cudaMemcpy( d_pid, h_pid, sizeof(int)*NUMPARTICLES_MAX_SIM,   cudaMemcpyHostToDevice );
    cudaDeviceSynchronize();
    cout<<" DONE! \n";


	Set_PSystem_State<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>( d_pos,d_Ornt,
			                                                             d_vel,
			                                                            d_Avel,
			                                                            d_ptype,d_pid,
			                                                            dram_position_com,
			                                                            dram_position_ornt,
			                                                            dram_velocity_com,
			                                                            dram_velocity_ang,
					                        		                	dram_ObjectType,
					                        		                	dram_P_ID);
	cudaDeviceSynchronize();


	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set System State: Set_PSystem_State Kernel "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

	 /* If we using Classic Verlet then old position is needed */
     if(SimInfo_C->Integration_Type==1)
	 {
		    Init_OldPos<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
			   		                  (dram_position_com,dram_position_com_old);
		    cudaDeviceSynchronize();
	 }


	 cudaFree(d_pos);

	 if(SimInfo_C->particle_type==1|| SimInfo_C->sphere_orient)
	 {
	   cudaFree(d_Ornt);
	 }
	 cudaFree(d_vel);

	 cudaFree(d_Avel);
	 cudaFree(d_ptype);
	 cudaFree(d_pid);

	 cout<<" DONE Freeing Memory ! \n";

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set System State: End MemFree "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

}
/*-----------------------------------------------------------------------------*/

void Device_Set_Tallys(float *h_Tallys)
{
	float *d_Tallys;

    cudaMalloc( (void**) &d_Tallys, sizeof(float)*6 );
    cudaDeviceSynchronize();
    cudaMemcpy( d_Tallys, h_Tallys, sizeof(float)*6, cudaMemcpyHostToDevice );
    cudaDeviceSynchronize();

	 Set_Tallys<<< 1,1 >>>(d_Tallys);
	 cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"Set Tallies: Set_Tallys Kernel "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }


	 cudaFree(d_Tallys);

}





/*---------------------------------------------------------------------------*/
               /* 1. Get the properties of all CUDA DEVICES */
/*---------------------------------------------------------------------------*/
void Get_DeviceInfo()
{


    cudaGetDeviceCount(&num_devices);

    cout<<"INFO-D: Total Devices: "<<num_devices<<endl;

    for( int i=0;i<num_devices; i++ )
	{

	    cudaGetDeviceProperties(&DevProp,i);
    	cout<<"INFO-D: Device: "<<device_num<<" with "<<DevProp.multiProcessorCount<<"SM"<<endl;
	    cout<<"INFO-D: Device:"<< i <<" , "<<DevProp.name<<endl;
	    cudaMemGetInfo(&a,&t);
	    cout<<" Clock (MHZ): "<<DevProp.clockRate/1000 << " Total Memory (MB): "
			  <<t*9.53674e-7<<" Free (MB): "<<a*9.53674e-7<< " Clock(MHZ): "<<
			   DevProp.memoryClockRate/1000<< " BUS (Bit) "<<
			                                      DevProp.memoryBusWidth<<endl;
	    if(i==device_num)
	    {
	     num_sm = DevProp.multiProcessorCount;
	    }
	}

    cout<<" \n";
}
/*---------------------------------------------------------------------------*/





/*---------------------------------------------------------------------------*/
   /* 1) ENTRY POINT Copies all data to the device sets kernel parameters*/
/*---------------------------------------------------------------------------*/
void Device_Set_SimulationData(  WOBJ*           h_WorldOBJ,
								 POBJ*           h_ParticleOBJ,
								 DOBJ*           h_DynamicOBJ,
								 SimulationInfo* h_SimInfo,
								 int             h_Num_WorldOBJ,
								 int             h_Num_ParticleOBJ,
								 int             h_Num_DynamicOBJ,
								 InitConfig      *h_PosConfig             )

{

	error_check =1;
	LogFileD.open( "../SimulationDevice.Log" );

	cout<<"INFO-D: Starting Device Configuration \n";
	device_num = h_PosConfig->use_device;
	Get_DeviceInfo();
	cudaDeviceSynchronize();



	/* Create a local copy of the SimOBJ */
	SimInfo_C = h_SimInfo;
	Device_use_multi_gpu = h_PosConfig->multi_gpu;

	NUMPARTICLES_MAX_SIM = h_SimInfo->Num_Particles;
	NUMDYOBJECTS         = h_Num_DynamicOBJ;
	NUMWOBJECTS          = h_Num_WorldOBJ;
	D_NUMPARTICLES       = NUMPARTICLES_MAX_SIM;/* Used for killing Silo */
	NUMPARTICLES_Current = NUMPARTICLES_MAX_SIM;/* Used for filling */


	cout<<"INFO-D: Total Particles = "<<NUMPARTICLES_MAX_SIM<<endl;
	cout<<"INFO-D: Size Types = "<<h_SimInfo->Num_ParticleObjects<<endl;

	for(int i=0; i< h_SimInfo->Num_ParticleObjects;i++)
	{
		cout<<"INFO-D: Num per type = "<<h_SimInfo->Num_ParticlesPerObject[i]<<endl;
	}


	/* Set thread info on the GPU */
    printf( "RUNLEVEL 0: Max SM occupy \n");

    int num_particles_SM = (int)ceil(NUMPARTICLES_MAX_SIM/(float)num_sm);

    /* Now get the number of blocks per SM */
    int num_blocks_SM =  (int)ceil(num_particles_SM/(float)h_PosConfig->threads_perBlock);
    printf(" blocks per SM %d \n",num_blocks_SM);

    threads_perBlock = h_PosConfig->threads_perBlock;
    int num_threads_block=h_PosConfig->threads_perBlock;

    /* block size is too big */
    if(num_particles_SM < h_PosConfig->threads_perBlock)
    {
	   num_threads_block = num_particles_SM; /* Single block is sufficient */
    }

    GlobalLaunch.dimBlock = make_uint3( num_threads_block,1,1);
    GlobalLaunch.dimGrid  = make_uint3( num_blocks_SM*num_sm,1,1 );

	printf("INFO-D: Launching %d Blocks with %d Threads: Total threads = %d \n",GlobalLaunch.dimGrid.x,
								 GlobalLaunch.dimBlock.x,GlobalLaunch.dimGrid.x*GlobalLaunch.dimBlock.x);


    /* Dynamically allocate Dynamics arrays */
	cudaSetDevice(0);
	cudaDeviceSynchronize();

	cout<<"INFO-D: Allocating Dynamic Arrays : \n";

    m_numGridCells = SimInfo_C->num_NNCells.x*SimInfo_C->num_NNCells.y
    		                                 *SimInfo_C->num_NNCells.z;


    cudaMalloc( (void**) &dram_position_com,  sizeof(float3)*NUMPARTICLES_MAX_SIM );
    cudaMalloc( (void**) &dram_velocity_com,  sizeof(float3)*NUMPARTICLES_MAX_SIM );
    cudaMalloc( (void**) &dram_accelrat_com,  sizeof(float3)*NUMPARTICLES_MAX_SIM );

    if (SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
    {
      cudaMalloc( (void**) &dram_position_ornt, sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
    }
    /* Need to store old pos for classic VV */
    if(SimInfo_C->Integration_Type==1)
    {
    	cudaMalloc( (void**) &dram_position_com_old,  sizeof(float3)*NUMPARTICLES_MAX_SIM );
    }

    cudaMalloc( (void**) &dram_velocity_ang,  sizeof(float3)*NUMPARTICLES_MAX_SIM );


    cudaMalloc( (void**) &dram_ObjectType  ,  sizeof(uint)*NUMPARTICLES_MAX_SIM );
    cudaMalloc( (void**) &dram_P_ID        ,  sizeof(int)*NUMPARTICLES_MAX_SIM );



    cudaMalloc( (void**) &dram_Lifter_Contact_Hist ,      sizeof(Contact_Info)*NUMPARTICLES_MAX_SIM );


    if(SimInfo_C->use_symmetry)
    {
      cudaMalloc( (void**) &dram_force_com_X ,  sizeof(float)*NUMPARTICLES_MAX_SIM );
      cudaMalloc( (void**) &dram_force_com_Y ,  sizeof(float)*NUMPARTICLES_MAX_SIM );
      cudaMalloc( (void**) &dram_force_com_Z ,  sizeof(float)*NUMPARTICLES_MAX_SIM );

      cudaMalloc( (void**) &dram_force_ang_X ,  sizeof(float)*NUMPARTICLES_MAX_SIM );
      cudaMalloc( (void**) &dram_force_ang_Y ,  sizeof(float)*NUMPARTICLES_MAX_SIM );
      cudaMalloc( (void**) &dram_force_ang_Z ,  sizeof(float)*NUMPARTICLES_MAX_SIM );
    }
    else
    {
      cudaMalloc( (void**) &force_PP    , sizeof(float3)   *NUMPARTICLES_MAX_SIM );
      cudaMalloc( (void**) &force_PP_ang, sizeof(float3)   *NUMPARTICLES_MAX_SIM );
    }


    /* NN arrays */
    /* Allocation of cell data */
    cudaMalloc( (void**) &m_dCellStart,         sizeof(uint)*m_numGridCells);
    cudaMalloc( (void**) &m_dCellEnd,           sizeof(uint)*m_numGridCells);

    cudaMalloc( (void**) &dram_GridParticleHash , sizeof(uint)*NUMPARTICLES_MAX_SIM );
    cudaMalloc( (void**) &dram_GridParticleIndex, sizeof(int)*NUMPARTICLES_MAX_SIM );


    cudaMalloc( (void**) &dram_force_com_Wall       , sizeof(float3)*NUMPARTICLES_MAX_SIM );
    cudaMalloc( (void**) &dram_force_com_Lifter       , sizeof(float3)*NUMPARTICLES_MAX_SIM );

    cudaMalloc( (void**) &dram_force_ang_Wall   , sizeof(float3)*NUMPARTICLES_MAX_SIM );
    cudaMalloc( (void**) &dram_force_ang_Lifter , sizeof(float3)*NUMPARTICLES_MAX_SIM );

    if(Get_WallPoints)
	{
     cudaMalloc( (void**) &dWallContactPoints, sizeof(float3)*NUMPARTICLES_MAX_SIM );
	}


    if(SimInfo_C->use_hist)
    {
       cudaMalloc( (void**) &dram_PP_Contact_Hist, sizeof(Contact_InfoPP)*NUMPARTICLES_MAX_SIM*8 );
       cudaMalloc( (void**) &dram_PP_Contact_Num, sizeof(int)*NUMPARTICLES_MAX_SIM );
    }

	float Tallys [6];

	for(int i=0;i<6;i++)
	{
	  Tallys[i]=0.0f;
	}

	float *d_Tally;
	cudaMalloc((void **)&d_Tally, sizeof(float)*6);
	cudaDeviceSynchronize();

	cudaMemcpy(d_Tally,Tallys, sizeof(float)*6,          cudaMemcpyHostToDevice );
	cudaDeviceSynchronize();

	Set_Tallys<<< 1,1 >>>(d_Tally);
	cudaDeviceSynchronize();
	cudaFree(d_Tally);

    cudaDeviceSynchronize();

	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Set Simulation Data: Dram memory alloc "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }


    cout<<"INFO-D: Copying Constant Data Device: \n";

	/* 1) Copy Objects to Device */
	cudaMemcpyToSymbol( SimParms,       h_SimInfo,     sizeof( SimulationInfo ) );
	cudaMemcpyToSymbol( WorldObject,    h_WorldOBJ,    sizeof(WOBJ)*h_Num_WorldOBJ );
	cudaMemcpyToSymbol( ParticleObject, h_ParticleOBJ, sizeof(POBJ)*h_SimInfo->Num_ParticleObjects);


	if(h_Num_DynamicOBJ>0)
	{
	 cudaMemcpyToSymbol( DynamicObject,  h_DynamicOBJ,  sizeof(DOBJ)*h_Num_DynamicOBJ);
	}

	if(!h_PosConfig->use_file)
	{
	if(h_PosConfig->grid_type<2)
	{
	 cudaMemcpyToSymbol( InitVel,        h_PosConfig->velocity,     sizeof(float3)*2);
	}
	else
	{
	  h_PosConfig->velocity[0]= make_float3(h_PosConfig->launch_Vel.x,h_PosConfig->launch_Vel.y,h_PosConfig->launch_Vel.z);
	  cudaMemcpyToSymbol( InitVel,        h_PosConfig->velocity,     sizeof(float3)*2);
	}
	cudaDeviceSynchronize();
	}
	else
	{
		  h_PosConfig->velocity[0]= make_float3(0.0,0.0,0.0f);
		  cudaMemcpyToSymbol( InitVel,        h_PosConfig->velocity,     sizeof(float3)*2);
	}

	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Set Simulation Data: Constant memory alloc "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }


	cout<<" GPU Malloc DONE!\n";




   if(SimInfo_C->use_symmetry)
   {
	/* Launch Kernel to set Initial parameters on device */
    Set_Init_Config_Device<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
	   		                  ( SimInfo_C->Num_ParticleObjects,
	   		                	SimInfo_C->Num_WorldObjects,
	   		                	NUMDYOBJECTS,

	   		                	dram_force_com_X, dram_force_com_Y, dram_force_com_Z,
	   		                	dram_force_com_Wall,dram_force_com_Lifter,
	   		                	dram_force_ang_X, dram_force_ang_Y, dram_force_ang_Z,
	   		                	dram_force_ang_Wall,dram_force_ang_Lifter,

	   		                	dram_velocity_com,
			                    dram_velocity_ang,
			                    dram_Lifter_Contact_Hist,
			                    dram_accelrat_com,dram_PP_Contact_Num ,0);
   }
   else
   {
		/* Launch Kernel to set Initial parameters on device */
	    Set_Init_Config_Device<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
		   		                  ( SimInfo_C->Num_ParticleObjects,
		   		                	SimInfo_C->Num_WorldObjects,
		   		                	NUMDYOBJECTS,

		   		                	force_PP,
		   		                	dram_force_com_Wall,dram_force_com_Lifter,
		   		                	force_PP_ang,
		   		                	dram_force_ang_Wall,dram_force_ang_Lifter,

		   		                	dram_velocity_com,
				                    dram_velocity_ang,
				                    dram_Lifter_Contact_Hist,
				                    dram_accelrat_com,dram_PP_Contact_Num,0 );
   }
    cudaDeviceSynchronize();

	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Set Simulation Data: Set_Init_Config_Device Kernel "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }

    /* Select Initial Position */
	if(!h_PosConfig->use_file)
	{
	  if( h_PosConfig->grid_type==0 )
	  {
	    Set_Position_DefaultGrid(h_PosConfig,h_ParticleOBJ);
	  }
	  else if( h_PosConfig->grid_type==1 )
	  {
		Set_Position_Random_Grid(h_PosConfig,h_ParticleOBJ);
	  }
	  else if( h_PosConfig->grid_type==2 )
	  {
		  Set_Position_Fill_Mill(h_ParticleOBJ,h_WorldOBJ,h_PosConfig->launch_Vel.y,h_PosConfig->fill_plane_start,h_PosConfig->fill_plane_end);
		  isFilling = true;
	  }
		if(SimInfo_C->Integration_Type==1)
		{
		    Init_OldPos<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
			   		                  (dram_position_com,dram_position_com_old);
		    cudaDeviceSynchronize();
		}

	}
	cudaDeviceSynchronize();

	LogFileD<<"INFO-D: All data copied to Device "<<std::endl;


    cudaMemGetInfo(&a,&t);
    cout<<" Clock (MHZ): "<<" Total Memory (MB): "
		  <<t*9.53674e-7<<" Free (MB): "<<a*9.53674e-7 <<endl;

    LogFileD.close();

	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Set Simulation Data: Error at end: Debug further "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }

}
/*-----------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
               /* Performs the simulation using non-symmetry */
/*---------------------------------------------------------------------------*/
void Device_DEM_UpdateSim()
{

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"DEM_UpdateSim: Start "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

	/* Dynamically add particles to simulation by updating number of particles
	 * and changing the downstream kernel launches */
	if(isFilling)
	{
		if((fill_counter>pack_clear_steps || NUMPARTICLES_Current==0) )
		{
		  fill_counter=0;

		  if(NUMPARTICLES_Current<NUMPARTICLES_MAX_SIM)
		  {
			Fill_State();

			if(SimInfo_C->Integration_Type==1)
			{
			    Init_OldPos<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
				   		                  (dram_position_com,dram_position_com_old);
			    cudaDeviceSynchronize();
			}

		  }
		  if(NUMPARTICLES_Current>=NUMPARTICLES_MAX_SIM)
		  {

			  printf("fill ended\n");
			  isFilling = false;
		  }
		}
		else
		{
			fill_counter++;
		}
	}
	else
	{
		NUMPARTICLES_Current = NUMPARTICLES_MAX_SIM;
	}

	//int sz = 1048576 * 100;
	//cudaDeviceSetLimit(cudaLimitPrintfFifoSize, sz);


    /*-----------------------------------------------------------------------*/
           /* 1. Calculate the GridIndex of Each Particle (Grid Hash) */
    /*-----------------------------------------------------------------------*/

    /* Calculate the Hash for All particles based on its Spatial Position */
    Kernel_SpatialDecomp_CalcPHash<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
    		                          ( dram_GridParticleHash, dram_GridParticleIndex,
    		                            dram_position_com,NUMPARTICLES_Current );
    /*-----------------------------------------------------------------------*/

    cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"DEM_UpdateSim: Hash "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }
    /*-----------------------------------------------------------------------*/
           /* 2. Sort Particles according to (Grid Hash) */
    /*-----------------------------------------------------------------------*/

    /* In index 0 is lowest and last is highest */
    SpatialDecomp_ThrustSort_ByHash(dram_GridParticleHash, dram_GridParticleIndex,NUMPARTICLES_Current);
    cudaDeviceSynchronize();
    /*-----------------------------------------------------------------------*/

    /* Reorder arrays based on hash */
    float3    *dSortedPos;
    float3    *dSortedVel;
    float3    *dSortedAcc;
    uint      *dSortedPType;
    int       *dSortedPID;

    Contact_Info    *dSortedLifter_Contact_Hist;

    Contact_InfoPP  *dSorted_PP_Contact_Hist;
    int             *dSorted_PP_Contact_Num;

    Quaterion *dSortedPosQ;
    float3    *dSortedVelQ;


    /* Old particle positions for VV */
    float3    *dSortedPosOld;
    if( SimInfo_C->Integration_Type==1 )
    {
        cudaMalloc( (void**) &dSortedPosOld  , sizeof(float3)   *NUMPARTICLES_Current );     /* 120MB */
    }

                 /* Allocation of PDA data */
    cudaMalloc( (void**) &dSortedPos  , sizeof(float3)   *NUMPARTICLES_Current );     /* 120MB */
    cudaMalloc( (void**) &dSortedVel  , sizeof(float3)   *NUMPARTICLES_Current );     /* 120MB */
    cudaMalloc( (void**) &dSortedAcc  , sizeof(float3)   *NUMPARTICLES_Current );     /* 120MB */

    if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
      cudaMalloc( (void**) &dSortedPosQ , sizeof(Quaterion)*NUMPARTICLES_Current ); /* 160MB */
	}

    cudaMalloc( (void**) &dSortedVelQ , sizeof(float3)   *NUMPARTICLES_Current );    /* 120MB */
    cudaMalloc( (void**) &dSortedPType, sizeof(uint)     *NUMPARTICLES_Current );        /* 12MB */
    cudaMalloc( (void**) &dSortedPID  , sizeof(int)      *NUMPARTICLES_Current );        /* 12MB */

    cudaMalloc( (void**) &dSortedLifter_Contact_Hist     , sizeof(Contact_Info)*NUMPARTICLES_Current );


    if(SimInfo_C->use_hist)
    {
     cudaMalloc( (void**) &dSorted_PP_Contact_Hist     , sizeof(Contact_InfoPP)*NUMPARTICLES_Current*32 );
     cudaMalloc( (void**) &dSorted_PP_Contact_Num     , sizeof(int)*NUMPARTICLES_Current );
    }

    cudaDeviceSynchronize();

    /* m_dGridParticleIndex contains the order which the dynamics arrays must be reordered */

    /* Sort arrays using temp copies */
    Kernel_SortArrays
      <<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
      ( dram_GridParticleIndex,
    	dSortedPos,
    	dSortedPosQ,
    	dSortedVel,
    	dSortedVelQ,
    	dSortedAcc,
    	dSortedPType,
    	dSortedPID,

        dSortedLifter_Contact_Hist,
        dSorted_PP_Contact_Hist,
        dSorted_PP_Contact_Num,
        dSortedPosOld,

        dram_position_com,
        dram_position_ornt,
        dram_velocity_com,
        dram_velocity_ang,
        dram_accelrat_com,
        dram_ObjectType,
        dram_P_ID,

        dram_Lifter_Contact_Hist,
        dram_PP_Contact_Hist,
        dram_PP_Contact_Num,

        dram_position_com_old,
        NUMPARTICLES_Current );

    cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"DEM_UpdateSim: Sort "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }
    /*-----------------------------------------------------------------------*/
                /* 4. Update the pos and vel based on sorting in 3. */
      /*-----------------------------------------------------------------------*/
      Kernel_SpatialDecomp_ReorderPDA
                  <<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
      		    ( dSortedPos,dSortedPosQ, dSortedVel,dSortedVelQ, dSortedAcc, dSortedPType,dSortedPID,
      		      dSortedLifter_Contact_Hist, dSorted_PP_Contact_Hist, dSorted_PP_Contact_Num,
      		      dSortedPosOld,
      		      dram_position_com,
      		      dram_position_ornt,
      		      dram_velocity_com,
                  dram_velocity_ang,
                  dram_accelrat_com,
                  dram_ObjectType, dram_P_ID,
                  dram_Lifter_Contact_Hist,
                  dram_PP_Contact_Hist,
                  dram_PP_Contact_Num,
                  dram_position_com_old,
                  NUMPARTICLES_Current );
     cudaDeviceSynchronize();
      /*-----------------------------------------------------------------------*/

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"DEM_UpdateSim: Reorder "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

    /* Free All but NNList  */
    cudaFree(dSortedPos);
    cudaFree(dSortedVel);
    cudaFree(dSortedAcc);

	if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
      cudaFree(dSortedPosQ);
	}
	if(SimInfo_C->Integration_Type==1)
	{
      cudaFree(dSortedPosOld);
	}

    cudaFree(dSortedVelQ);
    cudaFree(dSortedPType);
    cudaFree(dSortedPID);
    cudaFree(dSortedLifter_Contact_Hist);

    if(SimInfo_C->use_hist)
    {
          cudaFree(dSorted_PP_Contact_Hist);
          cudaFree(dSorted_PP_Contact_Num);
     }

   // printf(" %f   \n", Call_Time);

	/* Remove flagged particles from the sim */
	if(num_dead>0)
	{
		NUMPARTICLES_Current -= num_dead;

		if(SimInfo_C->Rotating_WObject)
		{
		  printf(" %f  %d \n", Call_Time, num_dead);
		}
		num_dead=0;

	    int num_particles_SM = (int)ceil(NUMPARTICLES_Current/(float)num_sm);

	    /* Now get the number of blocks per SM */
	   	int num_blocks_SM =  (int)ceil(num_particles_SM/(float)threads_perBlock);

	   	   int num_threads_block=threads_perBlock;

	   	   /* block size is too big */
	   	   if(num_particles_SM < threads_perBlock)
	   	   {
	   		   num_threads_block = num_particles_SM; /* Single block is sufficient */
	   	   }

	   	   GlobalLaunch.dimBlock = make_uint3( num_threads_block,1,1);
	   	   GlobalLaunch.dimGrid  = make_uint3( num_blocks_SM*num_sm,1,1 );

	   	   SimInfo_C->Num_Particles = NUMPARTICLES_Current;
	   	   cudaMemcpyToSymbol( SimParms,       SimInfo_C,     sizeof( SimulationInfo ) );
	   	   cudaDeviceSynchronize();
	}


    /*-----------------------------------------------------------------------*/
           /* 3. Bin data into Cells based on the Hash and index */
    /*-----------------------------------------------------------------------*/
    cudaMemset(m_dCellStart, 0xffffffff, m_numGridCells*sizeof(uint));
    cudaMemset(m_dCellEnd,   0xffffffff, m_numGridCells*sizeof(uint));
    cudaDeviceSynchronize();


    /* Bins data into cells - Creates from scratch at each step */
    Kernel_SpatialDecomp_BinData
      <<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
      ( dram_GridParticleHash,m_dCellStart, m_dCellEnd,NUMPARTICLES_Current);
    cudaDeviceSynchronize();
    /*-----------------------------------------------------------------------*/

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	cout<<"DEM_UpdateSim: BinData "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }







 uint  *dBroad_List;
 uint  *dNumNN;
 cudaDeviceSynchronize();

 /* Single code */
 if( !Device_use_multi_gpu )
 {

	  cudaMalloc( (void**) &dBroad_List, sizeof(uint)*NUMPARTICLES_Current*32 );
	  cudaMalloc( (void**) &dNumNN     , sizeof(uint)*NUMPARTICLES_Current    );
  }

 cudaDeviceSynchronize();


 if(!SimInfo_C->use_symmetry)
 {
    /*-----------------------------------------------------------------------*/
                       /* 5. Scan ALL NN PAIRS */
    /*-----------------------------------------------------------------------*/

      Kernel_BroadCollisionDetection_NonSymmetry
                         <<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock>>>
    		             ( m_dCellStart,m_dCellEnd,dBroad_List,dNumNN,dram_position_com,dram_ObjectType,dram_P_ID,NUMPARTICLES_Current);
      cudaDeviceSynchronize();

 	 if(error_check==1)
 	 {
 	    	cudaError_t errormsg=cudaGetLastError();
 	    	if(errormsg>0)
 	    	{
 	    	cout<<"DEM_UpdateSim: NN Search NonSymm "<<cudaGetErrorString(errormsg)<<endl;
 	    	 exit(1);
 	    	}
 	 }
    /*-----------------------------------------------------------------------*/
                       /* 6. Apply Particle Particle Forces */
    /*-----------------------------------------------------------------------*/


      if(SimInfo_C->particle_type==0)
	  {
			Kernel_ParticleInteraction_Spheres_NonSymmetry<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
				( dNumNN,dBroad_List,
				  dram_position_com,
				  dram_velocity_com,
				  dram_velocity_ang,
				  dram_ObjectType,
				  dram_P_ID,force_PP,force_PP_ang,NUMPARTICLES_Current   );
			cudaDeviceSynchronize();

		 	 if(error_check==1)
		 	 {
		 	    	cudaError_t errormsg=cudaGetLastError();
		 	    	if(errormsg>0)
		 	    	{
		 	    	cout<<"DEM_UpdateSim: Force spheres NonSymm "<<cudaGetErrorString(errormsg)<<endl;
		 	    	 exit(1);
		 	    	}
		 	 }
		  }
      else
      {

    	  if(SimInfo_C->use_hist)
    	  {
			Kernel_ParticleInteraction_Polyhedra_NonSymmetryHist<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
				( dNumNN,dBroad_List,
				  dram_position_com,
				  dram_position_ornt,
				  dram_velocity_com,
				  dram_velocity_ang,
				  dram_accelrat_com,
				  dram_ObjectType,
				  dram_P_ID,force_PP,force_PP_ang ,dram_PP_Contact_Hist, dram_PP_Contact_Num
				  ,NUMPARTICLES_Current);
    	  }
    	  else
    	  {
			Kernel_ParticleInteraction_Polyhedra_NonSymmetry<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
				( dNumNN,dBroad_List,
				  dram_position_com,
				  dram_position_ornt,
				  dram_velocity_com,
				  dram_velocity_ang,
				  dram_accelrat_com,
				  dram_ObjectType,
				  dram_P_ID,force_PP,force_PP_ang,
				  NUMPARTICLES_Current);
    	  }
			cudaDeviceSynchronize();

		 	 if(error_check==1)
		 	 {
		 	    	cudaError_t errormsg=cudaGetLastError();
		 	    	if(errormsg>0)
		 	    	{
		 	    	 cout<<"DEM_UpdateSim: Force poly NonSymm "<<cudaGetErrorString(errormsg)<<endl;
		 	    	 exit(1);
		 	    	}
		 	 }

      }

  }/* symmetry not verified */
  else
  {

	    /*-----------------------------------------------------------------------*/
	                       /* 5. Scan 1/2 NN PAIRS */
	    /*-----------------------------------------------------------------------*/

	      Kernel_BroadCollisionDetection_Symmetry
	                         <<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock>>>
	    		             ( m_dCellStart,m_dCellEnd,dBroad_List,dNumNN,dram_position_com,dram_ObjectType,dram_P_ID,NUMPARTICLES_Current);
	      cudaDeviceSynchronize();

	      PrintNN<<< 1,1>>>(dNumNN,dBroad_List,dram_P_ID);

	      cudaDeviceSynchronize();



		 	 if(error_check==1)
		 	 {
		 	    	cudaError_t errormsg=cudaGetLastError();
		 	    	if(errormsg>0)
		 	    	{
		 	    	 cout<<"DEM_UpdateSim: NN search Symm "<<cudaGetErrorString(errormsg)<<endl;
		 	    	 exit(1);
		 	    	}
		 	 }



        if(SimInfo_C->particle_type==0)
		{

			Kernel_ParticleInteraction_Spheres_Symmetry<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
					( dNumNN,dBroad_List,
					  dram_force_com_X,dram_force_com_Y,dram_force_com_Z,
					  dram_force_ang_X,dram_force_ang_Y,dram_force_ang_Z,

					  dram_position_com,
					  dram_velocity_com,
					  dram_velocity_ang,
					  dram_ObjectType,
					  dram_P_ID,NUMPARTICLES_Current);

				cudaDeviceSynchronize();

			 	 if(error_check==1)
			 	 {
			 	    	cudaError_t errormsg=cudaGetLastError();
			 	    	if(errormsg>0)
			 	    	{
			 	    	cout<<"DEM_UpdateSim: Force spheres Symm "<<cudaGetErrorString(errormsg)<<endl;
			 	    	 exit(1);
			 	    	}
			 	 }


		}
		else /* Polyhedra */
		{
			Kernel_ParticleInteractionPoly_Symmetry<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
				( dNumNN,dBroad_List,
				  dram_force_com_X,dram_force_com_Y,dram_force_com_Z,
		          dram_force_ang_X,dram_force_ang_Y,dram_force_ang_Z,

		          dram_position_com, dram_position_ornt,
		          dram_velocity_com, dram_velocity_ang,

		          dram_ObjectType,
		          dram_P_ID,NUMPARTICLES_Current);
			cudaDeviceSynchronize();

		 	 if(error_check==1)
		 	 {
		 	    	cudaError_t errormsg=cudaGetLastError();
		 	    	if(errormsg>0)
		 	    	{
		 	    	cout<<"DEM_UpdateSim: Force poly Symm "<<cudaGetErrorString(errormsg)<<endl;
		 	    	 exit(1);
		 	    	}
		 	 }

		}
   }

    if(NUMDYOBJECTS>0)
    {

	   VolumeObject_InteractionParticle<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>(
			     dram_force_com_Lifter,
			     dram_force_ang_Lifter,
			     dram_Lifter_Contact_Hist,
			     dram_position_com,
			     dram_position_ornt,
			     dram_velocity_com,
			     dram_velocity_ang,
	             dram_accelrat_com,
	             dram_ObjectType, dram_P_ID,NUMPARTICLES_Current);
	   cudaDeviceSynchronize();

	 	 if(error_check==1)
	 	 {
	 	    	cudaError_t errormsg=cudaGetLastError();
	 	    	if(errormsg>0)
	 	    	{
	 	    	 cout<<"DEM_UpdateSim: Lifter Inter "<<cudaGetErrorString(errormsg)<<endl;
	 	    	 exit(1);
	 	    	}
	 	 }
    }


	WorldObject_InteractionParticle<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>( Get_WallPoints, is_cylinder_rotation,
			                                                                        dram_force_com_Wall,
			                                                                        dram_force_ang_Wall,
			                                                                        dWallContactPoints,
			                                                                        dram_position_com,
			                                                                        dram_position_ornt,
			                                                                        dram_velocity_com,
			                                                                        dram_velocity_ang,
			                                                                        dram_accelrat_com,
                                                                                    dram_ObjectType,
                                                                                    dram_P_ID,NUMPARTICLES_Current                   );
	cudaDeviceSynchronize();

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	 cout<<"DEM_UpdateSim: Surface Interaction "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

  if(SimInfo_C->use_symmetry)
  {
      Integrate_Euler_Symmetry<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
            ( dram_force_com_X, dram_force_com_Y,dram_force_com_Z,
              dram_force_ang_X, dram_force_ang_Y, dram_force_ang_Z,
              dram_force_com_Wall,   dram_force_ang_Wall,
              dram_force_com_Lifter, dram_force_ang_Lifter,

              dram_position_com, dram_position_ornt,
              dram_velocity_com, dram_velocity_ang,
              dram_ObjectType, dram_P_ID,NUMPARTICLES_Current );
      cudaDeviceSynchronize();

	 	 if(error_check==1)
	 	 {
	 	    	cudaError_t errormsg=cudaGetLastError();
	 	    	if(errormsg>0)
	 	    	{
	 	    	 cout<<"DEM_UpdateSim: Integration Euler Symm "<<cudaGetErrorString(errormsg)<<endl;
	 	    	 exit(1);
	 	    	}
	 	 }

  }
  else /* Non symmetry*/
  {

    if(SimInfo_C->Integration_Type==0)
    {
      Integrate_Euler_NonSymmetry<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
            ( force_PP,          force_PP_ang,
              dram_force_com_Wall,   dram_force_ang_Wall,
              dram_force_com_Lifter, dram_force_ang_Lifter,

              dram_position_com, dram_position_ornt,
              dram_velocity_com, dram_velocity_ang,
              dram_ObjectType, dram_P_ID,NUMPARTICLES_Current );
      cudaDeviceSynchronize();

	 	 if(error_check==1)
	 	 {
	 	    	cudaError_t errormsg=cudaGetLastError();
	 	    	if(errormsg>0)
	 	    	{
	 	    	 cout<<"DEM_UpdateSim: Integration Euler Non Symm "<<cudaGetErrorString(errormsg)<<endl;
	 	    	 exit(1);
	 	    	}
	 	 }
    }
    else
    {
    	if(is_first)
    	{
        	Integrate_Verlet_NonSymmetryInit<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
        	            ( force_PP,force_PP_ang, dram_force_com_Wall,
        	              dram_force_com_Lifter,     dram_position_com_old,

        	              dram_position_com, dram_position_ornt,
        	              dram_velocity_com, dram_velocity_ang, dram_accelrat_com,
        	   	          dram_ObjectType, dram_P_ID);
        	      cudaDeviceSynchronize();
        	      is_first =false;
    	}

    	Integrate_Verlet_NonSymmetry<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
    	            ( force_PP,force_PP_ang, dram_force_com_Wall,
    	              dram_force_com_Lifter,     dram_position_com_old,

    	              dram_position_com, dram_position_ornt,
    	              dram_velocity_com, dram_velocity_ang, dram_accelrat_com,
    	   	           dram_ObjectType, dram_P_ID);
    	      cudaDeviceSynchronize();
    }

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	 cout<<"DEM_UpdateSim: Verlet NonSymm "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

  }

  cudaFree(dBroad_List);
  cudaFree(dNumNN);

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	 cout<<"DEM_UpdateSim: END "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

	 /* Flags particles to be removed from the sim
	  * and gives them a position such that the hash will be large */
     if( SimInfo_C->Kill_Particles)
     {
       stepc++;
       /* Dont check every step to save computations */
       if( stepc >= 1000 )
       {

    	  if(SimInfo_C->Rotating_WObject)
    	  {
        	 Remove_PL_Discharged_Particles<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>(dram_position_com,dram_P_ID,NUMPARTICLES_Current);
        	 cudaDeviceSynchronize();
    	  }
    	  else
    	  {


    	    Remove_Silo_Discharged_Particles<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>(dram_position_com,dram_P_ID,NUMPARTICLES_Current);
    	    cudaDeviceSynchronize();
    	  }

    	  /* Copy number killed to Host (Check if can do this without memcpy */
    	  int *d_kill;
    	  int  h_kill[2];
    	  cudaMalloc( (void**) &d_kill        , sizeof(int)*2   );
    	  Kill_copy<<<1,1>>>(d_kill);
    	  cudaDeviceSynchronize();
    	  cudaMemcpy( h_kill,        d_kill        ,sizeof(int)*2,
    	    		                                     cudaMemcpyDeviceToHost );
    	  num_dead=h_kill[0];
    	  cudaFree(d_kill);

    	  stepc = 0;
       }
     }

	 if(error_check==1)
	 {
	    	cudaError_t errormsg=cudaGetLastError();
	    	if(errormsg>0)
	    	{
	    	 cout<<"DEM_UpdateSim: KillParticles "<<cudaGetErrorString(errormsg)<<endl;
	    	 exit(1);
	    	}
	 }

	 Call_Time+=SimInfo_C->InitalDelta_t;

}
/*---------------------------------------------------------------------------*/






/*---------------------------------------------------------------------------*/
/*    Returns the new positions to the host to render per time-step/frame    */
/*---------------------------------------------------------------------------*/
void Device_Get_P_Positions( float3 *Pos, Quaterion *Quart_ornt,uint *PType,
		                     int *hP_ID                  )
{
	float3    *d_pos;
	Quaterion *d_Quart_ornt;
	uint      *d_ptype;
	int       *d_PID;

	if( NUMPARTICLES_Current > 0 )
	{
	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Get Positions : Error Start "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }



	cudaMalloc( (void**) &d_pos        , sizeof(float3)*NUMPARTICLES_Current    );
	cudaMalloc( (void**) &d_ptype      , sizeof(uint)*NUMPARTICLES_Current      );
	cudaMalloc( (void**) &d_PID        , sizeof(uint)*NUMPARTICLES_Current      );

	if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
	  cudaMalloc( (void**) &d_Quart_ornt , sizeof(Quaterion)*NUMPARTICLES_Current );
	}
	cudaDeviceSynchronize();


      Get_Particle_PosArray<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
    		                      ( d_pos, d_Quart_ornt, d_ptype, d_PID,
    		                       dram_position_com, dram_position_ornt,
    		                        dram_ObjectType, dram_P_ID, NUMPARTICLES_Current );
	  cudaDeviceSynchronize();


    cudaMemcpy( Pos,        d_pos        ,sizeof(float3)*NUMPARTICLES_Current,
    		                                     cudaMemcpyDeviceToHost );
    cudaMemcpy( PType,      d_ptype      ,sizeof(uint)*NUMPARTICLES_Current,
    		                                     cudaMemcpyDeviceToHost );

    cudaMemcpy( hP_ID,       d_PID        ,sizeof(int)*NUMPARTICLES_Current,
    		                                     cudaMemcpyDeviceToHost );

    if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
      cudaMemcpy( Quart_ornt, d_Quart_ornt ,sizeof(Quaterion)*NUMPARTICLES_Current,
    		                                     cudaMemcpyDeviceToHost );
	}
    cudaDeviceSynchronize();

    cudaFree( d_pos );

    if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
	{
     cudaFree( d_Quart_ornt );
	}

    cudaFree( d_ptype );
    cudaFree( d_PID );

	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Get Positions : Error End "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }
	}
	else /* Start up during filling */
	{
	 hP_ID[0] =-2;
	}


}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*    Returns the new positions to the host to render per time-step/frame    */
/*---------------------------------------------------------------------------*/
void Device_Get_System_State( float3 *h_pos, Quaterion *h_Ornt, float3 *h_vel,
		                      float3 *h_Avel,float3 *h_acc, uint *h_type,
		                      int *h_pid)
{
	float3    *d_pos;
	float3    *d_vel;
	//float3    *d_acc;
	Quaterion *d_Quart_ornt;
	float3    *d_Avel;
	uint      *d_ptype;
	int       *d_PID;


	cudaMalloc( (void**) &d_pos        , sizeof(float3)*NUMPARTICLES_MAX_SIM    );

	if( SimInfo_C->particle_type==1)
	{
	  cudaMalloc( (void**) &d_Quart_ornt , sizeof(Quaterion)*NUMPARTICLES_MAX_SIM );
	}

	cudaMalloc( (void**) &d_vel        , sizeof(float3)*NUMPARTICLES_MAX_SIM    );
	//cudaMalloc( (void**) &d_acc        , sizeof(float3)*NUMPARTICLES_MAX_SIM    );
	cudaMalloc( (void**) &d_Avel        , sizeof(float3)*NUMPARTICLES_MAX_SIM    );
	cudaMalloc( (void**) &d_ptype      , sizeof(uint)*NUMPARTICLES_MAX_SIM      );
	cudaMalloc( (void**) &d_PID        , sizeof(int)*NUMPARTICLES_MAX_SIM      );
	cudaDeviceSynchronize();


      Get_SystemState<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
    		                      ( d_pos,d_Quart_ornt, d_vel,d_Avel,
    		                        d_ptype, d_PID,
    		                        dram_position_com, dram_position_ornt,
    		                        dram_velocity_com, dram_velocity_ang,
    		                        dram_ObjectType,dram_P_ID               );
	  cudaDeviceSynchronize();


    cudaMemcpy( h_pos,        d_pos        ,sizeof(float3)*NUMPARTICLES_MAX_SIM,
    		                                     cudaMemcpyDeviceToHost );

	if( SimInfo_C->particle_type==1 )
	{
    cudaMemcpy( h_Ornt, d_Quart_ornt ,sizeof(Quaterion)*NUMPARTICLES_MAX_SIM,
      		                                     cudaMemcpyDeviceToHost );
	}


    cudaMemcpy( h_vel,        d_vel        ,sizeof(float3)*NUMPARTICLES_MAX_SIM,
    		                                     cudaMemcpyDeviceToHost );

//    cudaMemcpy( h_acc,        d_acc        ,sizeof(float3)*NUMPARTICLES_MAX_SIM,
//    		                                     cudaMemcpyDeviceToHost );

    cudaMemcpy( h_Avel,       d_Avel        ,sizeof(float3)*NUMPARTICLES_MAX_SIM,
    		                                     cudaMemcpyDeviceToHost );


    cudaMemcpy( h_type,      d_ptype      ,sizeof(uint)*NUMPARTICLES_MAX_SIM,
    		                                     cudaMemcpyDeviceToHost );
    cudaMemcpy( h_pid,       d_PID        ,sizeof(int)*NUMPARTICLES_MAX_SIM,
    		                                     cudaMemcpyDeviceToHost );
    cudaDeviceSynchronize();

    cudaFree( d_pos );

    if(SimInfo_C->particle_type==1)
	{
      cudaFree( d_Quart_ornt );
	}

    cudaFree( d_vel );
    cudaFree( d_Avel );
    //cudaFree( d_acc );
    cudaFree( d_ptype );
    cudaFree( d_PID );
}
/*---------------------------------------------------------------------------*/


void Device_Get_P_Velocity( float3 *H_Vel, float3 *H_Rvel, int Flag,
                            uint *h_ptype, int *h_pid                )
{

	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Get Velocity : Error Start "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }

	float3 *d_vel;
	float3 *d_velR;

	uint      *d_ptype;
	int       *d_PID;

	cudaMalloc( (void**) &d_vel  , sizeof(float3)*NUMPARTICLES_Current );

	if(Flag==1)
	{
	  cudaMalloc( (void**) &d_velR , sizeof(float3)*NUMPARTICLES_Current );
	}

	cudaMalloc( (void**) &d_ptype      , sizeof(uint)*NUMPARTICLES_Current      );
	cudaMalloc( (void**) &d_PID        , sizeof(int)*NUMPARTICLES_Current      );
	cudaDeviceSynchronize();

	  Get_Particle_VelArray<<< GlobalLaunch.dimGrid, GlobalLaunch.dimBlock >>>
	                                                               ( d_vel,d_velR,d_ptype,d_PID,
	                                                            	 Flag, dram_velocity_com, dram_velocity_ang,
	                                                            	 dram_ObjectType,d_PID,NUMPARTICLES_Current);
	cudaDeviceSynchronize();

	cudaMemcpy( H_Vel,  d_vel  ,sizeof(float3)*NUMPARTICLES_Current,
			                                          cudaMemcpyDeviceToHost );

	if(Flag==1)
	{
	cudaMemcpy( H_Rvel, d_velR ,sizeof(float3)*NUMPARTICLES_Current,
			                                          cudaMemcpyDeviceToHost );
	}

    cudaMemcpy( h_ptype,      d_ptype      ,sizeof(uint)*NUMPARTICLES_Current,
    		                                     cudaMemcpyDeviceToHost );
    cudaMemcpy( h_pid,       d_PID        ,sizeof(int)*NUMPARTICLES_Current,
    		                                     cudaMemcpyDeviceToHost );


	cudaDeviceSynchronize();
    cudaFree( d_vel );
    cudaFree( d_ptype );
    cudaFree( d_PID );

    if(Flag==1)
    {
      cudaFree( d_velR );
    }

	 if(error_check==1)
	 {
		cudaError_t errormsg=cudaGetLastError();
		if(errormsg>0)
		{
		 cout<<"Get Velocity : Error End "<<cudaGetErrorString(errormsg)<<endl;
		 exit(1);
		}
	 }
}





void Device_Get_W_Forces(float3 *WallForces,float3 *WallContact_Point)
{

      cudaMemcpy( WallForces,   dram_force_com_Wall        ,sizeof(float3)*NUMPARTICLES_MAX_SIM,
            		                                     cudaMemcpyDeviceToHost );

      cudaMemcpy( WallContact_Point,   dWallContactPoints        ,sizeof(float3)*NUMPARTICLES_MAX_SIM,
            		                                     cudaMemcpyDeviceToHost );
      cudaDeviceSynchronize();
}


void Start_Bin()
{
	start_bin=true;
}


void Device_Reset_Energy(int Num)
{

	ResetEnergy<<< 1,1 >>>(Num);
	cudaDeviceSynchronize();
}


void Device_Get_Energy(float *Energy)
{
	float *d_energy;

	cudaMalloc( (void**) &d_energy  , sizeof(float)*NumPhysicsTallies);
	cudaDeviceSynchronize();

	GetEnergy<<< 1,1 >>>(d_energy,NumPhysicsTallies);
	cudaDeviceSynchronize();

	cudaMemcpy( Energy,     d_energy     ,sizeof(float)*NumPhysicsTallies,
  		                                     cudaMemcpyDeviceToHost );
    cudaDeviceSynchronize();
    cudaFree( d_energy );

    if(error_check==1)
    {
    	cudaError_t errormsg=cudaGetLastError();
    	 if(errormsg>0)
    	 {
    	   cout<<"Get Energy: "<<cudaGetErrorString(errormsg)<<endl;
    	   exit(1);
    	 }
    }
}



/*---------------------------------------------------------------------------*/
/*                    Passes updated Dynamic object List                     */
/*---------------------------------------------------------------------------*/
void Device_Update_DObjectPostions( DOBJ* h_DynamicOBJ)
{

	cudaMemcpyToSymbol( DynamicObject, h_DynamicOBJ,    sizeof(DOBJ)*NUMDYOBJECTS );
	cudaDeviceSynchronize();
}
/*-----------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*                    Passes updated Dynamic object List                     */
/*---------------------------------------------------------------------------*/
void Device_Update_WorldObjectPostions( WOBJ *h_WorldOBJ)
{

	cudaMemcpyToSymbol( WorldObject, h_WorldOBJ,    sizeof(WOBJ)*NUMWOBJECTS );
	cudaDeviceSynchronize();
}
/*-----------------------------------------------------------------------------*/



void drum_rotation(bool cylinder_rotation)
{
	is_cylinder_rotation = cylinder_rotation;
}




/*-----------------------------------------------------------------------------*/
            /* Passes a new object removing the last object */
/*-----------------------------------------------------------------------------*/
void Device_OpenHatch( WOBJ* h_WorldOBJ,int h_Num_WorldOBJ)
{

	cudaMemcpyToSymbol( WorldObject,    h_WorldOBJ,    sizeof(WOBJ)*h_Num_WorldOBJ );
	cudaDeviceSynchronize();
    Hatch<<< 1,1 >>>();
    cudaDeviceSynchronize();

	printf("Hatch Opened\n");

}
/*-----------------------------------------------------------------------------*/

void Device_AddForce( )
{

    AddForce<<< 1,1 >>>();
    cudaDeviceSynchronize();
}


void Device_SubForce( )
{

    SubForce<<< 1,1 >>>();
    cudaDeviceSynchronize();
}


void Device_ZeroForce( )
{

    ZeroForce<<< 1,1 >>>();
    cudaDeviceSynchronize();
}





/*-----------------------------------------------------------------------------*/
                      /* Frees allocated arrays*/
/*-----------------------------------------------------------------------------*/
void Device_Clean()
{


	if(!SimInfo_C->use_symmetry)
	{
	  cudaFree(dram_force_com_X);
	  cudaFree(dram_force_com_Y);
	  cudaFree(dram_force_com_Z);

	  cudaFree(dram_force_ang_X);
	  cudaFree(dram_force_ang_Y);
	  cudaFree(dram_force_ang_Z);
	}
	else
	{
	    cudaFree(force_PP);
	    cudaFree(force_PP_ang);
	}

	cudaFree(dram_force_com_Wall);
	cudaFree(dram_force_com_Lifter);

	cudaFree(dram_force_ang_Wall);
	cudaFree(dram_force_ang_Lifter);

	if(Get_WallPoints)
	{
	  cudaFree(dWallContactPoints);
	}

	cudaFree(m_dCellStart);
	cudaFree(m_dCellEnd);

	cudaFree(dram_GridParticleHash);
	cudaFree(dram_GridParticleIndex);

	cudaFree(dram_Lifter_Contact_Hist);
	cudaFree(dram_ObjectType);
	cudaFree(dram_P_ID);
	cudaFree(dram_accelrat_com);
	cudaFree(dram_position_com);

	if(SimInfo_C->particle_type==1 || SimInfo_C->sphere_orient)
    {
	 cudaFree(dram_position_ornt);
    }
	cudaFree(dram_velocity_ang);
	cudaFree(dram_velocity_com);


    if(SimInfo_C->use_hist)
    {
    	cudaFree(dram_PP_Contact_Hist);
    	cudaFree(dram_PP_Contact_Num);
    }

    if(SimInfo_C->Integration_Type==1)
    {
    	cudaFree(dram_position_com_old);
    }

    if(!Device_use_multi_gpu)
    {

    }


	cudaDeviceReset();

	printf(" Freed GPU Memory!\n");
}



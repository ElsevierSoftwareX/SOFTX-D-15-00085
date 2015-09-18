/* Author Nicolin Govender: govender.nicolin@gmail.com */
/* Revision: 5/5/2014 */

/*---------------------------------------------------------------------------*/
       /* This File contains the containers used for data in the code */
/*---------------------------------------------------------------------------*/

#ifndef KSIMULATIONOBJECTS_H_
#define KSIMULATIONOBJECTS_H_

#include <string>
#include <vector_types.h>
#include "../Utilities/Common/Types.h"

using namespace std;


struct __builtin_align__(16) Contact_Info
{
    int num_contact, obj_id, snum, cont_type;
};


struct __builtin_align__(16) Contact_InfoPP
{
    int obj_id, cont_type, contact_face, AorB;
};


struct  Contact_InfoPP2
{
    uint    obj_id      [32];
//    int8_t  cont_type   [32];
//    int8_t  contact_face[32];
//    int8_t AorB        [32];
};

enum Output_Type
{
  user       = 0,
  step       = 1,
  mill_EndRev= 2,
  restart    = 3

};

enum Sim_Type
{
  general  = 0,
  ballmill = 1,
  silo     = 2,
};


enum Sim_Particle_Type
{
  spheres      = 0,
  polyhedra    = 1,
  sphere_poly  = 2
};



struct Mill_Data
{
  int    power_output_revs;
  int    total_revs;
  float  mill_RPM;
  double RadPerStep;
  int    Num_Steps_PerRev;
  int    Rot_PerStep;
  int    Current_StepNumber;
};

struct Silo_Data
{
  bool   measure_flow;
  int    flow_steps;
  bool   kill_particle;
  bool   is_hatch_open;
  bool   manual_hatch;
  int    flow_count;
  int    num_particles_rem;
  float  hatch_height;
  int    flow_rate;

};



struct launchParms
{
	int  num_gpus;
	int  nthreads;
	dim3 dimGrid;
	dim3 dimBlock;
};



/*------------------------------------------------------*/
/*        1. Information for Host and Device            */
/*------------------------------------------------------*/
struct SimulationInfo
{

  /* Particle Info*/
  int particle_type;
  int Num_Particles;
  int Num_ParticleObjects;
  int *Num_ParticlesPerObject;
  int Num_WorldObjects;
  int TotnumCells;

  bool Rotation;
  bool Mode2D;
  bool EnergyCalc;
  bool isMillSim;
  bool Kill_Particles;


  float  InitalDelta_t;
  float  MillRotVel;
  float Global_VisDamp;    /* Overall  Damping    */
  float Roll_Res;
  float Silo_Kill_Height;
  float Vel_Limit;
  float DObject_CylinderBoundR;


  int3   num_NNCells;
  float3 Force_Field;
  float3 worldOrigin;
  float3 cellSize;
  float3 cent;
  float3 Vel_Limit_Bounds;
  float3 max_size;

  Sim_Type  Simulation_Type;
  int       Integration_Type;

  bool    use_symmetry;
  bool    sphere_orient;
  bool    dobject_hasVertex;

  bool    Rotating_WObject;
  bool    use_hist;

  int    unit_test;


};
/*------------------------------------------------------*/



/*------------------------------------------------------*/
struct InitConfig
{
   int3   num;
   float3 space;
   float3 start;
   float3 p_size[16];
   bool   is_spheres;
   bool   use_file;
   int    grid_type;

   float3    launch_Vel;
   float3   fill_plane_start;
   float3   fill_plane_end;

   float3 velocity[12];
   float3 position[12];
   float3 ang_vel[12];
   bool   DebugFlag;
   int    use_device;
   bool   multi_gpu;
   int    threads_perBlock;


};
/*------------------------------------------------------*/

struct NNGrid
{
   int3   num;
   float3 size;
   float3 origin;
};


struct OpenGLOBJ
{
  int   FPS;
  bool  render;
  bool  ortho;
  int   Debug_Lines;
  int   write_intermFile;
  bool  is_runLimit;
  float run_time;
  float total_SimTime;
  int   color_type;
  float color_bin;
  bool  is_energy_calc;
  int   energy_calc_steps;
  bool  is_surface_tally;
  int   surface_tally_calc_steps;
  bool  is_energy_bin;
  int   energy_bin_calc_steps;
  int   output_format;

  bool  world_surface_Shading[4][32];
};
/*------------------------------------------------------*/


/*------------------------------------------------------*/
           /* Geometric Data */
/*------------------------------------------------------*/

/* World Object Type Description */
enum SurfaceType
{ plane     = 0,
  cylinder  = 1
};

/* Holds vertexes that define the edge */
struct Edge_Def
{
 uint point_vindex [2];
};


struct Plane
{
  float3 normal;
  float3 centroid;
};


struct Surface
{
  int    num_vertex;
  int    vertex_Order [8];
  float  area;

  float3 normal;
  float3 centroid;

  float Tol;
};

struct Edge
{
    float3 Point;
    float3 dir;
};



struct PP_Parms
{
    float Kn;                /* Spring   Normal     */
    float Cn;                /* Damping  Normal     */
    float Fric_static;       /* Friction Tangential */
    float Fric_kinetic;
};

/*------------------------------------------------------*/
                 /* Particle Object */
/*------------------------------------------------------*/
struct POBJ
{
    float    radius; /* cm */
    float    mass;   /* kg */

	/* Polyhedra Particle Info not needed for spheres
	 * check how to exclude with c++                 */
	float3   COM;
	int      num_vertex;
	float3   Local_Vertex [32];/* Rotate on GPU*/
	int      num_edges;
	Edge_Def edge         [32];
	int      num_faces;
	Surface  face         [32];
	float    InertiaT     [9]; /* Row of T*/
	float3   vertex       [32];

	/* Particle Interaction */
	PP_Parms  PP[6];

	/* Surface Interaction */
	float surface_Kn;           /* Spring   Normal     */
	float surface_Cn;           /* Damping  Normal     */
	float surface_Fric_static;         /* Friction Tangential */
	float surface_Fric_kinetic;

	/* Dynamic Interaction */
	float Dsurface_Kn;           /* Spring   Normal     */
	float Dsurface_Cn;           /* Damping  Normal     */
	float Dsurface_Fric_static;         /* Friction Tangential */
	float Dsurface_Fric_kinetic;

};
/*------------------------------------------------------*/



/*------------------------------------------------------*/
              /* Dynamic World Object */
/*------------------------------------------------------*/
struct D_OBJ_HostLocal
{
  float3 Local_Vertex[8];
};


struct DOBJ
{

  /* Bounding Object Information */
  uint    btype;  /* Cylinder/Sphere */
  float3  Axis;   /* Orientation ( along z axis) */

  float   boundR; /* Radius of encompassing cylinder */
  float3  COM;    /* Center of Mass */

  /* Global Geometric Information */
  int      num_vertex;       /* Number of vertcies */
  float3   vertex       [8]; /* Rotated on CPU */
  int      num_faces;     /* Number of planar surfaces */
  Surface  faces     [3]; /* Normal, Centroid etc */

  int       num_edges;
  Edge_Def  edge[4];

  float3  velocity;

  bool is_attached_DWorld;
  bool is_translating;

};
/*------------------------------------------------------*/


/*------------------------------------------------------*/
                   /* World Data */
/*------------------------------------------------------*/

struct Macro_Cylinder
{
  float  height;
  float  radius;
  bool   has_top_cap;
  bool   has_bot_cap;
  float3 normal_cyl;
  float3 normal_top_cap;
  float3 normal_bot_cap;
  float3 center_top_cap;
  float3 center_bot_cap;
  float3 AXIS;

};



/* A world object can be made up of planar surfaces
    *                OR
    * second order surfaces which are MACROS         */
struct WOBJ
{
  SurfaceType surface_type;
  int         num_surfaces;
  Surface     surfaces    [16];
  int         num_vertex;
  float3      vertex      [16];
  bool        is_rotating;

  /* MACRO surface */
  Macro_Cylinder cyl_geo;
};





/*------------------------------------------------------*/
                   /* TODO FUTURE */
/*------------------------------------------------------*/

/* String description for world */
struct Descp_WorldOBJ
{
 string name  [10];
 string descp [10];
};

/* String description */
struct Descp_ParticleOBJ
{
  string name  [16];
  string descp [16];
};


struct ENVOBJ
{

  bool gravity;
  bool air_res;
  bool global_damping;
};

















#endif /* KSIMULATIONOBJECTS_H_ */

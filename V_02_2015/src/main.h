
/*---------------------------------------------------------------------------*/
/*                             BLAZE-DEM V2.0
 -----------------------------------------------------------------------------

 * --------------------------
 *     Complete Features
 * --------------------------
 *       1. INPUT (KSimulationData.h)
   * World Input FileIO
   * World    Object Input FileIO
   * Particle Object Input FileIO
   --------------------------
         2. Data Structures
   * Simulation Information: Host/Device (KSimulationObjects.h)
   --------------------------
         3. Graphics
   * (Main.cpp) and (Graphics.h)
   * Only the Position and Rotation is required
   --------------------------
         4. Host/Device
   * Communication Via Class: KInterface
 */

/* Author: Nicolin Govender
 * Revision 1: 22/9/2012
 * Revision 2: 30/10/2012
 * Revision 3: 4/11/2012 : Added Grid Hashing
 * Revision 4: 1/1/2013  : NNGrid Via KInput
 * Revision 4: 2/6/2013  : Clean up for Release
 * Revision 4: 2/6/2013  : Clean up for Paper
 * */

/* Feature List


/*---------------------------------------------------------------------------*/
                    /* REQUIRED INCLUDE FILES*/
/*---------------------------------------------------------------------------*/
                     /* System C++ */
#include <iostream>
#include <stdlib.h>
#include <sstream>


                      /* Vector types */
#include <vector_types.h>
#include "Utilities/Host/Functions.h"
#include "Utilities/Host/Operators.h"


                      /* Project Includes */
#include "Input/KSimulationData.h"

#include "Interface/KDevice.h"


const double PI=3.14159265;

/*---------------------------------------------------------------------------*/
                     /* Pointers to Classes*/
/*---------------------------------------------------------------------------*/

KSimulationData *m_KSimulationData;  /* Input Data*/

KSimulationData *m_KSimulationData2;  /* Input Data*/

KDevice         *m_KDevice;          /* Device methods */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
                     /* Data Storage Objects */
/*---------------------------------------------------------------------------*/
SimulationInfo   SimInfo;
WOBJ            *m_KWorldObject;
WOBJ            *m_KWorldObject_Initial;

POBJ            *m_KParticleObject;

DOBJ            *m_KDynamicObject;
DOBJ            *m_KDynamicObject_Initial;
OpenGLOBJ       m_OpenGL;

//D_OBJ_HostLocal  *m_KDynamicObject_Host;

InitConfig       m_InitPos;

int              m_num_KWorldObjects;
int              m_num_KParticleObjects;
int              m_num_KDynamicObjects;
int              cuda_device=0;
bool             use_multi_gpu=false;

string ProjectFolder;
string WorldName;
/*---------------------------------------------------------------------------*/

float energy[6];


bool Translate_VObj=false;


Mill_Data m_Mill_SimData;
Silo_Data m_Silo_SimData;

int m_Sim_Type;


int m_Current_StepNumber=0;


string FnamesSnap[50];

int FPS=10;

int Revoution_number=1;


int read_initialPos_file=0;

Quaterion dq;

bool energy_Bin=false;

int num_energy_BinEvents=15238108 ;
int Energy_BinsCount_N[100000];
int Energy_BinsCount_S[100000];
int Energy_BinsCount_T[100000];
float Energy_BinSize=5E-6;

bool post_process=false;

                   /* Local Variables */
int    frame           = 0;
double simtime_elapsed = 0.0;
bool   m_ColorLevels   = false;
char   fps[256];

float3 GLScale;

ofstream LogFile;
ofstream ResultFile;

ofstream PDA;
fstream  PDAFIN;
Quaterion axisangle;

float Start_Energy=0.0f;

int m_WallForceSteps=0;
int m_WallForceCount=0;

bool Get_WallForces_H=true;


int num_WrotSeps=0;


double Rotation_Angle=0.0;

int Init_NumParticles;


/*---------------------------------------------------------------------------*/
     /* Particle Data Arrays (PDA) store in Global memory on HOST */
     float3    *m_d_Particle_Pos  ; /* <--DEVICE */
     Quaterion *m_d_Particle_Quart; /* <--DEVICE */
     int       *m_d_P_ID          ; /* <--DEVICE */
     uint       *m_d_Particle_Type ; /* <--DEVICE */
     float3    *m_d_Dynamic_Pos ; /* <--DEVICE */

     float3    *m_d_Particle_Vel  ; /* <--DEVICE */
     float3    *m_d_Particle_RVel  ; /* <--DEVICE */

     float3    *m_d_Particle_Acc  ; /* <--DEVICE */

     float3    *m_colorP        ; /* Local Host */

     float3 *m_dc_WallForces;
     float3 *m_dc_WallContacts;

     int m_sim_particle_type;

/*---------------------------------------------------------------------------*/

     bool m_colorbyVel=false;
     float offsetE=0.0f;

/*--------------------------------------------------------------------------------*/

   ofstream  OFs;


Quaterion quart_axisAngleH(Quaterion Q1)
{
	Quaterion q;
	float angle = (2.0f*acos(Q1.w));
	float coeff=0.0f;
	if(angle>0.0f)
	{
	 coeff = 1.0f/sin(angle*0.5f);
	}
	 q.w = angle*(180.0f/(2.0f*asin(1.0)));
	q.x = Q1.x*coeff;
	q.y = Q1.y*coeff;
	q.z = Q1.z*coeff;


	return q;
}


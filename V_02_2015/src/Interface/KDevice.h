/*
 * KDevice.h
 *
 *  Created on: Oct 18, 2012
 *      Author: nicolin
 */


#ifndef KDEVICE_H_
#define KDEVICE_H_
typedef unsigned int uint;
#include "../DataStructures/KSimulationObjects.h"
#include "../Device/DeviceInterface.h"

using namespace std;

class KDevice {
public:
	KDevice ( WOBJ           *worldObject,       POBJ       *particleObject, DOBJ       *dynamicObject,
			  int             num_worldObjects,  int         num_particleObjects,int num_dynamicObjects,
			  SimulationInfo *simInfo,           InitConfig *h_PosConfig );

	void clean(){Device_Clean();};

	/* Action Methods */
	void IDevice_World_Interaction    ()         { Device_World_Interaction() ;};
	void IDevice_World_InteractionS    ()         { Device_World_InteractionS() ;};
	void IDevice_Body_Body_Interaction()         { Device_Body_Body_Interaction() ;};
	void IDevice_DEM_UpdateSim()         { Device_DEM_UpdateSim() ;};

	void IDevice_Mill_reset_energy(int value){ Device_Reset_Energy(value);};

	void IDevice_UpdateVolObjectPositions( DOBJ* h_DynamicOBJ){Device_Update_DObjectPostions(  h_DynamicOBJ);};

	void IDevice_UpdateWorldObjects( WOBJ *h_WorldOBJ){Device_Update_WorldObjectPostions( h_WorldOBJ);};

	void IDevice_OpenHatch( WOBJ* h_WorldOBJ,int h_Num_WorldOBJ){Device_OpenHatch( h_WorldOBJ,h_Num_WorldOBJ);};

	void IDevice_End_Packing(SimulationInfo *SimInfo, WOBJ *WorldOBJ){End_Packing(SimInfo,WorldOBJ);};

	void IDevice_Set_Tallys(float *h_Tallys){Device_Set_Tallys(h_Tallys);};

	void IDevice_drum_rotation(bool cylinder_rotation){drum_rotation(cylinder_rotation);};
	/* Get Methods */

	void IDevice_Get_System_State( float3 *h_pos,Quaterion *h_Ornt, float3 *h_vel,float3 *h_Avel, float3 *h_acc,

	                               uint *h_type, int *h_pid                   )
	{Device_Get_System_State( h_pos,h_Ornt, h_vel, h_Avel, h_acc,

            h_type, h_pid);};


	void IDevice_Get_Positions ( float3 *h_pos, Quaterion *h_Quart_ornt, uint *h_type,
			                     int *P_ID )
	                           { Device_Get_P_Positions(h_pos,h_Quart_ornt,h_type,P_ID) ;};

	void IDevice_Get_Particle_Velocity(float3 *H_Vel, float3 *H_Rvel, int flag,uint *h_type,
            int *P_ID)
	                           {Device_Get_P_Velocity(H_Vel,H_Rvel,flag,h_type,P_ID);};



	void IDevice_Get_W_Forces(float3 *WallForces,float3 *WallContact_Point){Device_Get_W_Forces(WallForces,WallContact_Point);};

	void IDevice_Get_Energy(float *Energy){Device_Get_Energy(Energy);};


	/* Set Methods */


	void IDevice_Set_ParticleSystem_State( float3 *h_pos, Quaterion *h_Ornt,
			                              float3 *h_vel, float3 *h_Avel, float3 *h_acc,
			                               uint *h_type, int *h_pid )

	{ Device_Set_System_State( h_pos, h_Ornt, h_vel,h_Avel,h_acc, h_type, h_pid); };


	void IDevice_Set_ParticleSystem_Pos( float3 *h_pos, Quaterion *h_Ornt,
			                             uint *h_type, int *h_pid )
	                    { Device_Set_Pos( h_pos,h_Ornt, h_type, h_pid); };


	void IDevice_Set_ParticleSystem_PosV( float3 *h_pos,Quaterion *h_Ornt, float3 *h_vel,
			                              float3 *h_Avel,
			                             uint *h_type, int *h_pid )
	                    { Device_Set_Pos_Velocity( h_pos,h_Ornt, h_vel,h_Avel, h_type, h_pid);};



	void IStart_Bin(){Start_Bin();};

	void IDevice_AddForce(){Device_AddForce();};
	void IDevice_SubForce(){Device_SubForce();};
	void IDevice_ZeroForce(){Device_ZeroForce();};


	virtual ~KDevice();
};

#endif /* KDEVICE_H_ */

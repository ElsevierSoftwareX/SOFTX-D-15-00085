/*
 *
 *  Created on: Sep 7, 2012
 *      Author: nicolin
 */

#include "../DataStructures/KSimulationObjects.h"

#ifndef DeviceInterface_H_
#define DeviceInterface_H_


                     /* Set Methods */

          /* Sets up the Simulation on the Device */
void Device_Set_SimulationData ( WOBJ* h_WorldOBJ,     POBJ* h_ParticleOBJ,
		                         DOBJ* h_DynamicOBJ,   SimulationInfo* h_SimInfo,
		                         int h_Num_WorldOBJ,   int h_Num_ParticleOBJ,
		                         int h_Num_DynamicOBJ, InitConfig *h_PosConfig);


void Device_Set_Pos( float3  *h_pos,  Quaterion *h_Ornt,
		             uint    *h_ptype, int *h_pid        );


void Device_Set_Pos_Velocity( float3    *h_pos,  Quaterion *h_Ornt,
		                      float3 *h_vel, float3 *h_Avel,
		                      uint      *h_ptype, int *h_pid      );


void Device_Set_System_State( float3    *h_pos, Quaterion *h_Ornt,  float3 *h_vel,float3 *h_Avel, float3 *h_acc,

                              uint      *h_ptype, int    *h_pid);


void Device_Set_Tallys(float *h_Tallys);


                            /* Get Methods */

/* Primary Parameters */
void Device_Get_System_State( float3    *h_pos, Quaterion *h_Ornt,
        float3 *h_vel,
        float3 *h_Avel,
        float3 *h_acc,
        uint  *h_ptype,
        int *h_pid);


void Device_Get_P_Positions( float3 *Pos, Quaterion *Quart_ornt,
		                     uint *h_ptype, int *h_pid           );


void Device_Get_P_Velocity  ( float3 *H_Vel, float3 *H_Rvel, int Flag,
		                      uint *h_ptype, int *h_pid                );

void Device_Get_P_Accelration (float3 *H_Acc, uint *h_ptype, int *h_pid);


/* Secondary Parameters */
void Device_Get_W_Forces(float3 *WallForces,float3 *WallContact_Point);

void Device_Get_Energy(float *Energy);

void Device_Get_NN_List(uint *H_NNList);



                          /* Action Methods */

/* Removes the last world Object */
void Device_Update_DObjectPostions( DOBJ* h_DynamicOBJ);
void Device_Update_WorldObjectPostions( WOBJ* h_WorldOBJ);

void Device_OpenHatch( WOBJ* h_WorldOBJ,int h_Num_WorldOBJ );
void Device_Clean();
void Device_Reset_Energy(int num_values);
void End_Packing(SimulationInfo *SimInfo, WOBJ *WorldOBJ);

void drum_rotation(bool cylinder_rotation);
void Start_Bin();

void Device_AddForce( );
void Device_SubForce( );
void Device_ZeroForce( );



/*---------------------------------------------------------------------------*/
          /* Functionality methods (Host requests Kernel Launch) */
/*---------------------------------------------------------------------------*/
void Device_World_Interaction();
void Device_World_InteractionS();
void Device_Body_Body_Interaction();
void Device_DEM_UpdateSim();

/*---------------------------------------------------------------------------*/

#endif /* DeviceInterface_H_ */

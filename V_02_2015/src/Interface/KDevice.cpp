/*
 * KDevice.cpp
 *
 *  Created on: Oct 18, 2012
 *      Author: nicolin
 */

#include "KDevice.h"
#include "../DataStructures/KSimulationObjects.h"


KDevice::KDevice( WOBJ* worldObject, POBJ* particleObject,DOBJ* dynamicObject,
		          int num_worldObjects, int num_particleObjects,int num_dynamicObjects,
		          SimulationInfo *simInfo, InitConfig *h_PosConfig )
{
	Device_Set_SimulationData( worldObject, particleObject,dynamicObject, simInfo,
			                        num_worldObjects, num_particleObjects,num_dynamicObjects,h_PosConfig);

}

KDevice::~KDevice()
{
	// TODO Auto-generated destructor stub
}


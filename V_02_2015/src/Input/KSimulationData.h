/*
 * KWorld.h
 *
 *  Created on: Oct 13, 2012
 *      Author: nicolin
 *      govender.nicolin@gmail.com
 *      This class reads the ALL the information for the simulation.
 *      1) Environment Objects ( Surfaces, containers)
 *      2) Particle Information
 */
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include "../DataStructures/KSimulationObjects.h"




using namespace std;

#ifndef KSimulationData_H_
#define KSimulationData_H_

class KSimulationData {
public:

	KSimulationData( string ProjectFolder, string WorldName );
	virtual ~KSimulationData();

    int       get_isDebug          () { return m_is_debug; };
	int        get_readFile        () { return m_readFile; };


	int        get_particleType      () { return m_Sim_ParticleType;    };
	int        get_simType      () { return m_Sim_type;    };


	int        get_num_WorldObjects      () { return m_num_WorldObjects;    };
	int        get_num_ParticleObjects   () { return m_num_ParticleObjects; };
	int        get_num_DYParticleObjects () { return m_num_DynamicObjects; };
	int        get_num_Lifters          () { return m_num_DObjectsPerType; };
	int        get_num_Particles       () { return m_total_NumParticles; };
    int*       get_num_ParticlesPerObject   () { return m_num_ParticlesPerObject; };

	WOBJ*      get_worldObject           () { return m_WorldObjectsList;    };
	POBJ*      get_particleObject        () { return m_ParticleObjectsList; };
	DOBJ*      get_dynamicObject         () { return m_DynamicObjectsList; };

	D_OBJ_HostLocal*      get_dynamicObjectLocal         () { return m_KDynamicObject_HostList; };



	float      get_delta_t             () { return m_delta_t; };
	float3     get_gravity             () { return m_force_field; };

	int        get_isRotation          () { return m_isRotation; };
	float      get_RollRes             () { return m_RollRes; };
	float      get_GlobalDamp             () { return m_GlobalDamping; };
	float      get_ColDampPP             () { return m_CollDamp_PP; };
	float      get_ColDampPS             () { return m_CollDamp_PS; };
	float      get_ColDampPD             () { return m_CollDamp_PD; };

	Mill_Data  get_millObject          () { return m_millObject; };
	Silo_Data  get_siloObject          () { return m_siloObject; };

	string     get_filename         () {return inputfile;};

	InitConfig get_InitPosGrid         () { return m_InitPosGrid; };
	NNGrid     get_NNGrid              () { return m_NNGRID; };
	float3     get_worldSize           () { return m_worldSize; };
	float     get_velLimit             () { return m_vel_Limit; };

	OpenGLOBJ  get_OpenGLObject        () { return m_OpenGLObject; };

	void Output_SystemState(Output_Type Otype, int revflag, int Flag_COM);
	void Read_FromFile(string Fname);


private:

	 /* File info */
	string    m_DataPath;
	fstream   WObjectsF;
	fstream   PObjectsF;
	fstream   DObjectsF;

	ofstream   LogFile;

	int m_readFile;

	/* member variables */
	string    m_WorldTitle;
	string    m_WorldDesc;
	int       m_num_WorldObjects;
	int       m_num_ParticleObjects;
	int       m_num_DynamicObjects;
	int       m_num_EnvironmentOptions;
	int       m_num_OpenGLOptions;


    float m_vel_Limit;



	/* Pointers to Objects */
	WOBJ     *m_WorldObjectsList;
	POBJ     *m_ParticleObjectsList;
	DOBJ     *m_DynamicObjectsList;

	D_OBJ_HostLocal  *m_KDynamicObject_HostList;

	OpenGLOBJ m_OpenGLObject;

    /* Simulation Parameters */
	int       m_total_NumParticles;
	int       m_num_ParticlesPerObject [16];
	float     m_delta_t;
	float3    m_force_field;

	int       m_isRotation;
	float     m_RollRes;
	float     m_GlobalDamping;

	int  m_is_debug;

	float     m_CollDamp_PP;
	float     m_CollDamp_PS;
	float     m_CollDamp_PD;


	int m_Sim_type;
	int m_Sim_ParticleType;

	Mill_Data m_millObject;
	Silo_Data m_siloObject;

    double tc_PS;
    double tc_PP;
    double tc_PD;

    string inputfile;

	double m_Mill_Speed ;
	int     m_num_DObjectsPerType;

	int num_revs;

	/* Grid Information */
    InitConfig m_InitPosGrid;
    NNGrid     m_NNGRID;
    float3     m_worldSize;

    /* Dummy */
	Descp_ParticleOBJ m_ParticleObjectDescp;
	Descp_WorldOBJ    m_WorldObjectDescp;

	int m_ForceSpec;

	double COR[6];


	void Print_WObject (WOBJ * OBJ);
	void Print_PObject (POBJ * OBJ);
	void ReadWObject   (int i);
	void ReadPObject_Sphere   (int i);



	void ReadPObject_Poly     (int i);

	void ReadDObject   (int i);

	void Make_Particle_LocalInfo(POBJ * Pobj);
	void Make_DObject_LocalInfo(DOBJ * Pobj, int index);

	/* Calculates all spring constants based on contact time */
	void Set_SpringConstantsSurface_TC(int i, double COR);
	void Set_SpringConstantsDSurface_TC(int i, double COR);
	void Set_SpringConstantsParticle_TC(int i,int j, double COR_i,double COR_j);

	/* User specified spring constant */
	void Set_SpringConstantsSurface_KN(int i, double COR);
	void Set_SpringConstantsDSurfaceKN(int i, double COR);
	void Set_SpringConstantsParticleKN(int i,int j, double COR_i,double COR_j);


	void Make_Particle_LocalInfoBB(POBJ * Pobj);

	void Make_New_Lifter(float3 about,int index, int d_index, float angle);
	void Repeat_Lifters();

};

#endif /* KSimulationData_H_ */

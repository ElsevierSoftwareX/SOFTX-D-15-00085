/*
 * KSimulationData.cpp
 *
 *  Created on: Oct 13, 2012
 *      Author: nicolin govender
 *      govender.nicolin@gmail.com
 *      20/9/2013: Clean up for first release
 *      8/6/2014: Adding Debug POS to Sim file
 *      28/1/2014 Adding SimType for silo
 */


#include "KSimulationData.h"

#include <vector_types.h>
#include <math.h>
#include "../DataStructures/Quart.h"
#include "../DataStructures/Host_Helpers.h"
#include "../Utilities/Host/Operators.h"



/*---------------------------------------------------------------------------*/
                            /* Constructor*/
/* Entry point reads all data based info in the World File */
/*---------------------------------------------------------------------------*/
KSimulationData::KSimulationData( string ProjectFolder, string WorldName )
{

  fstream WorldF;

  string dumS;
  int    dumI;

  m_DataPath      = "../Projects/"+ProjectFolder +"/";

  /* Open Logfile */
  LogFile.open( ( m_DataPath+"/"+ WorldName +"/Simulation.Log").c_str());

  LogFile<<"Installation Directory:"<<endl;

  LogFile<<endl;
  LogFile<<"   ********************************************************************"<<endl;
  LogFile<<"   *                  Welcome to BLAZE-DEM V1.1                       *"<<endl;
  LogFile<<"   *                     *dynamic world objects                       *"<<endl;
  LogFile<<"   *                Build: (7/2015) GCC 4.8 CUDA 7.0 Linux            *"<<endl;
  LogFile<<"   *           CUDA Compute 3.0 and above is required                 *"<<endl;
  LogFile<<"   *                                                                  *"<<endl;
  LogFile<<"   *                                                                  *"<<endl;
  LogFile<<"   *         (Nicolin Govender govender.nicolin@gmail.com)            *"<<endl;
  LogFile<<"   ********************************************************************"<<endl;
  LogFile<<endl;



  string filepath = m_DataPath +"World/"+ WorldName+".World";

  std::cout<<filepath<<endl;

  LogFile<<endl;

  LogFile<<"-------------------------------------------------------------------"<<endl;
  LogFile<<"                      Starting Data Read                           "<<endl;
  LogFile<<endl;

  LogFile<<"INFO-INPUT: Opening Simulation File : "<<filepath<<endl;

  WorldF.open(filepath.c_str());

  filepath = "";

  if(!WorldF.is_open())
  {
	  LogFile<<"!Error-INPUT: Simulation File file not found!"<<endl;
	  exit(1);
  }
  else
  {
	/* Read All data from KSimulationData */
	WorldF>>dumS>>m_WorldTitle;
	WorldF>>dumS>>m_ForceSpec;
	WorldF>>dumS>>m_Sim_type;
	WorldF>>dumS>>m_Sim_ParticleType;
	WorldF>>dumS>>m_delta_t;

	if(m_WorldTitle=="UNIT_TEST")
	{
     m_is_debug=1;
	}
	else
	{
	 m_is_debug=0;
	}


	if(m_ForceSpec==0)
	{
	  LogFile<<"INFO-INPUT: Using Auto Force Model : |contact time and COR| "<<endl;
	}
	else if(m_ForceSpec==1)
	{
	  LogFile<<"INFO-INPUT: Using Manual Force Model :| Kn and COR| "<<endl;
	}
	else
	{
		  LogFile<<"!Error-INPUT: Invalid Force Model (Line 2) !"<<endl;
		  exit(1);
	}


	if(m_Sim_type==general)
	{
	  LogFile<<"INFO-INPUT: General mode dynamics only "<<endl;
	}
	else if(m_Sim_type==ballmill)
	{
	  LogFile<<"INFO-INPUT: Mill Simulation mode "<<endl;
	}
	else if(m_Sim_type==silo)
    {
		  LogFile<<"INFO-INPUT: Silo Simulation mode "<<endl;
	}
	else
	{
		  LogFile<<"!Error-INPUT: Invalid Simulation Mode (Line 3) !"<<endl;
		  exit(1);
	}

	if(m_Sim_ParticleType==spheres)
	{
	  LogFile<<"INFO-INPUT: Spherical Particles "<<endl;
	}
	else if(m_Sim_ParticleType==polyhedra)
	{
	  LogFile<<"INFO-INPUT: Polyhedral Particles  "<<endl;
	}
	else
	{
		  LogFile<<"!Error-INPUT: Invalid Particle Type (Line 4) !"<<endl;
		  exit(1);
	}

	WorldF>>dumS;
	WorldF>>dumS;
	WorldF>>dumS;
	WorldF>>dumS;
	WorldF>>dumS;

	WorldF>>dumS>>m_num_WorldObjects;

	if(m_num_WorldObjects>0)
	{
	  LogFile<<"INFO-INPUT: "<<m_num_WorldObjects<<" world objects "<<endl;
	}
	else
	{
	  LogFile<<"!Error-INPUT: Invalid number of world objects (Line 12) !"<<endl;
	  exit(1);
	}


	m_WorldObjectsList = new WOBJ[m_num_WorldObjects];
    LogFile<<endl;

	string WOBJ_Fname;



	for (int i=0;i<m_num_WorldObjects;i++)
	{

	   WorldF>>dumI>>WOBJ_Fname;

	   if (dumI==99)
	   {
		 cout<<" Rotating world object "<<endl;
		 m_WorldObjectsList[i].is_rotating = true;
		 WorldF>>dumS>>m_WorldObjectsList[i].cyl_geo.AXIS.x>>m_WorldObjectsList[i].cyl_geo.AXIS.y>>m_WorldObjectsList[i].cyl_geo.AXIS.z;
		 WorldF>>dumS>>m_millObject.mill_RPM>>dumS>>m_millObject.total_revs;

		 m_WorldObjectsList[i].cyl_geo.radius = max(max(m_WorldObjectsList[i].cyl_geo.AXIS.x,m_WorldObjectsList[i].cyl_geo.AXIS.y),m_WorldObjectsList[i].cyl_geo.AXIS.z);
		 m_WorldObjectsList[i].cyl_geo.AXIS/=length(m_WorldObjectsList[i].cyl_geo.AXIS);

	   }
	   else
	   {
		 m_WorldObjectsList[i].is_rotating = false;
	   }

       filepath = m_DataPath +"SurfaceObjects/"+WOBJ_Fname+".WOBJ";

       LogFile<<"INFO-INPUT: Opening Object "<<filepath<<" "<<i+1<<" of "
    		                               <<m_num_WorldObjects<<endl;

       WObjectsF.open(filepath.c_str());

	   if(!WObjectsF.is_open())
	   {
		 LogFile<<"!Error-KSimulationData: WObject Not found!"<<endl;
		 exit(1);
	   }
	   else
	   {
         ReadWObject(i);
	   }

	   WObjectsF.close();
	}

	LogFile<<"INFO-INPUT: Reading world objects: Done "<<endl;

	LogFile<<endl;
	LogFile<<endl;

	WorldF>>dumS;
	WorldF>>dumS;
	WorldF>>dumS;



	WorldF>>dumS>>m_num_ParticleObjects;

	if(m_num_ParticleObjects>0)
	{
		LogFile<<"INFO-INPUT: "<<m_num_ParticleObjects<<" Particle Objects"<<endl;
		LogFile<<endl;
	}
	else
	{
	  LogFile<<"!Error-INPUT: Invalid number of Particle objects!"<<endl;
	  exit(1);
	}



		m_ParticleObjectsList = new POBJ[m_num_ParticleObjects];
		string POBJ_Fname;

		for (int i=0;i<m_num_ParticleObjects;i++)
		{

		   WorldF>>dumI>>POBJ_Fname>>m_num_ParticlesPerObject[i];
	       filepath = m_DataPath +"ParticleObjects/"+POBJ_Fname+".POBJ";

	       LogFile<<"INFO-INPUT: Opening Object "<<filepath<<" "<<i+1<<" of "
	    		                               <<m_num_ParticleObjects<<endl;

	       PObjectsF.open(filepath.c_str());

		   if(!PObjectsF.is_open())
		   {
			 LogFile<<"!Error-INPUT: PObject Not found!"<<endl;
			 exit(1);
		   }
		   else
		   {
			 if(m_Sim_ParticleType==0)
			 {
	          ReadPObject_Sphere(i);
			 }
			 else if(m_Sim_ParticleType==1)
			 {
	          ReadPObject_Poly(i);
			 }

		   }

		   PObjectsF.close();
		}
		LogFile<<"INFO-INPUT: Reading particle objects: Done "<<endl;



		LogFile<<"INFO-KSimData: Calculating Spring Constants "<<endl;
		/* Set PP Force Matrix*/

		for (int i=0;i<m_num_ParticleObjects;i++)
		{

			for (int j=0;j<m_num_ParticleObjects;j++)
			{
				if(m_ForceSpec==0)
				{
		          Set_SpringConstantsParticle_TC( i, j, COR[i], COR[j]);
				}

				if(m_ForceSpec==1)
				{

		         Set_SpringConstantsParticleKN( i, j, COR[i], COR[j]);
				}
			}
		}

		LogFile<<"INFO-INPUT: Calculating Spring Constants: Done "<<endl;
		LogFile<<endl;

		WorldF>>dumS;
		WorldF>>dumS;
		WorldF>>dumS;



		/* Read Dynamic Objects */
		WorldF>>dumS>>m_num_DynamicObjects>>dumS>>m_num_DObjectsPerType;

		if(m_num_DynamicObjects==0)
		{
			LogFile<<"INFO-INPUT: NO Dynamic Objects"<<endl;
			LogFile<<endl;
		}


        if( m_num_DynamicObjects>0 )
        {
        	/* num Repeat */
        	if(m_num_DObjectsPerType>0)
        	{
        	   m_DynamicObjectsList = new DOBJ[m_num_DynamicObjects*m_num_DObjectsPerType];
        	   m_KDynamicObject_HostList = new D_OBJ_HostLocal[m_num_DynamicObjects*m_num_DObjectsPerType];
        	}
        	else/* Manual mode*/
        	{
        		m_DynamicObjectsList = new DOBJ[m_num_DynamicObjects];
        		m_KDynamicObject_HostList = new D_OBJ_HostLocal[m_num_DynamicObjects];
        	}
				LogFile<<"INFO-INPUT: Reading "<<m_num_DynamicObjects<<" Dynamic Objects"<<endl;
				LogFile<<endl;



				string DYOBJ_Fname;

				for (int i=0;i<m_num_DynamicObjects;i++)
				{

				   WorldF>>dumI>>DYOBJ_Fname;

				   //if(m_WorldObjectsList[0].is_rotating)
				   //{
				     if(dumI==99)
				     {
					   m_DynamicObjectsList[i].is_attached_DWorld = false;
					   m_DynamicObjectsList[i].is_translating = false;
					   cout<<"DObject is static "<<endl;
				     }
				     else if(dumI==98)
				     {
				       m_DynamicObjectsList[i].is_attached_DWorld = false;
				 	   m_DynamicObjectsList[i].is_translating = true;
				 	   WorldF>>dumS>>m_DynamicObjectsList[i].velocity.x>>m_DynamicObjectsList[i].velocity.y>>m_DynamicObjectsList[i].velocity.z;
				       cout<<"DObject is translating "<<endl;
				 	 }
				     else
				     {
					   m_DynamicObjectsList[i].is_attached_DWorld = true;
					   m_DynamicObjectsList[i].is_translating = false;
				     }
				   //}

				   if(m_Sim_type==ballmill)
				   {
			         filepath = m_DataPath +"LifterObjects/"+DYOBJ_Fname+".DOBJ";
				   }
				   else
				   {
					   filepath = m_DataPath +"VolumeObjects/"+DYOBJ_Fname+".DOBJ";
				   }

			       LogFile<<"INFO : Opening Object "<<filepath<<" "<<i+1<<" of "
			    		                               <<m_num_DynamicObjects<<endl;

			       DObjectsF.open(filepath.c_str());

				   if(!DObjectsF.is_open())
				   {
					 LogFile<<"!Error-KSimData: DYObject Not found!"<<endl;
					 exit(1);
				   }
				   else
				   {
			         ReadDObject(i);

				   }


				   DObjectsF.close();
				}

				LogFile<<"INFO: Reading Dynamic Objects: Done "<<endl;
				LogFile<<""<<endl;


				if(m_num_DObjectsPerType>0)
				        	{
					Repeat_Lifters();
				        	}

        }


		LogFile<<"INFO-INPUT: Reading Simulation INFO "<<endl;
		LogFile<<""<<endl;

		WorldF>>dumS;
		WorldF>>dumS;
		WorldF>>dumS;

		WorldF>>dumS>>m_total_NumParticles;
		WorldF>>dumS>>m_force_field.x>>m_force_field.y>>m_force_field.z;

		WorldF>>dumS;

		/* Get running total */
		int sum=0;
		for(int i=0; i<m_num_ParticleObjects;i++)
		{
		  sum+=m_num_ParticlesPerObject[i];
		}

		if(sum==m_total_NumParticles)
		{
		  LogFile<<"INFO-INPUT: "<<m_total_NumParticles<<" particles "<<endl;
		}
		else
		{
		  LogFile<<"!Error-INPUT: Invalid number of particle objects !"<<endl;
		  exit(1);
		}

		/* Sane check for delta */
		if(m_delta_t>1E-1 || m_delta_t<0.0)
		{
		  LogFile<<"!Error-INPUT: Invalid time step "<<endl;
		  exit(1);
		}

		/* Sane check for force */
//		if( length(m_force_field)<0.10f)
//		{
//		  LogFile<<"!Error-INPUT: Invalid force field  "<<endl;
//		  exit(1);
//		}



		/* Rotation  */

		WorldF>>dumS>>m_isRotation;
		WorldF>>dumS>>m_RollRes;
		WorldF>>dumS>>m_GlobalDamping;
		WorldF>>dumS>>m_vel_Limit;


		/* Sane check for rotation */
		if( m_RollRes >=0.0f)
		{
		  LogFile<<"!Info-INPUT: Using rolling resistance  "<<m_RollRes<<endl;
		}
		else
		{
		  LogFile<<"!Error-INPUT: Invalid rolling res  "<<endl;
		  exit(1);
		}



		WorldF>>dumS;
	    WorldF>>dumS;
	    WorldF>>dumS;

	    /* Read NN Grid*/
		WorldF>>dumS>>m_NNGRID.origin.x>>m_NNGRID.origin.y>>m_NNGRID.origin.z;
		WorldF>>dumS>>m_worldSize.x>>m_worldSize.y>>m_worldSize.z;
		WorldF>>dumS>>m_NNGRID.size.x>>m_NNGRID.size.y>>m_NNGRID.size.z;

		WorldF>>dumS;
	    WorldF>>dumS;
		WorldF>>dumS;

		/* Read initial  Pos data */


		WorldF>>dumS>>m_readFile;

		/* Read a file name */
		if(m_readFile==1)
		{

		  WorldF>>dumS>>inputfile;
		  m_InitPosGrid.use_file=true;
		  m_InitPosGrid.grid_type=0;
		}
		else /* Use initial packing */
		{

		  m_InitPosGrid.use_file=false;

		  WorldF>>dumS>>m_InitPosGrid.grid_type;
		  if(m_InitPosGrid.grid_type<2)
		  {
		  WorldF>>dumS>>m_InitPosGrid.num.x>>m_InitPosGrid.num.y>>m_InitPosGrid.num.z;
		  WorldF>>dumS>>m_InitPosGrid.start.x>>m_InitPosGrid.start.y>>m_InitPosGrid.start.z;
		  WorldF>>dumS>>m_InitPosGrid.space.x>>m_InitPosGrid.space.y>>m_InitPosGrid.space.z;
		  WorldF>>dumS>>m_InitPosGrid.velocity[0].x>>m_InitPosGrid.velocity[0].y>>m_InitPosGrid.velocity[0].z;
		  }
		  else
		  {

			  WorldF>>dumS>>m_InitPosGrid.launch_Vel.x>>m_InitPosGrid.launch_Vel.y>>m_InitPosGrid.launch_Vel.z;
			  if(m_Sim_type!=ballmill)
			  {
			    WorldF>>dumS>>m_InitPosGrid.fill_plane_start.x>>m_InitPosGrid.fill_plane_start.y>>m_InitPosGrid.fill_plane_start.z;

			    WorldF>>dumS>>m_InitPosGrid.fill_plane_end.x>>m_InitPosGrid.fill_plane_end.y>>m_InitPosGrid.fill_plane_end.z;
			  }
			printf("Once filling is complete press p to output state \n");

			if(COR[0]>0.35)
			{
			 printf("!!Error: COR must be <=0.35 for filling \n");
			 exit(1);
			}
			if( (m_total_NumParticles%2)!=0)
			{
				printf("!!Error: Num Particles must be even \n");
				exit(1);
			}
		  }
		}

		WorldF>>dumS;
	    WorldF>>dumS;
		WorldF>>dumS;

		WorldF>>dumS>>dumI;

		if(dumI>0)
		{
		  m_OpenGLObject.render = true;
		}
		else
		{
			m_OpenGLObject.render = false;
		}

		if(m_OpenGLObject.render)
		{
			WorldF>>m_OpenGLObject.color_type;
			if(m_OpenGLObject.color_type>0)
			{
				WorldF>>m_OpenGLObject.color_bin;
			}
		}
		WorldF>>dumS>>m_OpenGLObject.ortho;
		WorldF>>dumS>>m_OpenGLObject.FPS;
		WorldF>>dumS>>m_OpenGLObject.Debug_Lines;
		WorldF>>dumS>>m_OpenGLObject.write_intermFile;

		if(m_OpenGLObject.write_intermFile>0)
		{
		  WorldF>>m_OpenGLObject.output_format;
		}

		WorldF>>dumS>>m_OpenGLObject.total_SimTime;


		int lim;
		WorldF>>dumS>>lim;
		if(lim==1)
		{
		  WorldF>>m_OpenGLObject.run_time;
		  m_OpenGLObject.is_runLimit=true;
		}
		else
		{
			m_OpenGLObject.is_runLimit=false;
		}

		WorldF>>dumS>>m_OpenGLObject.energy_calc_steps;

		if(m_OpenGLObject.energy_calc_steps>0)
		{
		  m_OpenGLObject.is_energy_calc = true;
		}
		else
		{
			m_OpenGLObject.is_energy_calc = false;
		}


		WorldF>>dumS>>m_OpenGLObject.surface_tally_calc_steps;

		if(m_OpenGLObject.surface_tally_calc_steps>0)
		{
		  m_OpenGLObject.is_surface_tally = true;
		}
		else
		{
		  m_OpenGLObject.is_surface_tally = false;
		}

		/* End of output section */
		WorldF>>dumS;




		/* Now read simulation specific flags*/


		if(m_Sim_type==ballmill)
		{


		    WorldF>>dumS;
			WorldF>>dumS;

		  WorldF>>dumS>>m_millObject.power_output_revs;
		  WorldF>>dumS>>m_millObject.total_revs;
		  WorldF>>dumS>>m_millObject.mill_RPM;
		  WorldF>>dumS;

		}
		else if(m_Sim_type==silo)
		{

		    WorldF>>dumS;
			WorldF>>dumS;

		  int dumI;
          WorldF>>dumS>>dumI;

          if(dumI==0)
          {
            m_siloObject.manual_hatch=false;
          }
          else
          {
        	  m_siloObject.manual_hatch=true;
          }

          WorldF>>dumS>>dumI>>m_siloObject.flow_steps;
          if(dumI==0)
          {
        	 m_siloObject.measure_flow =false;
          }
          else
          {
        	  m_siloObject.measure_flow =true;
          }


          WorldF>>dumS>>m_siloObject.kill_particle;
          WorldF>>dumS;
		}




		LogFile<<"INFO: Reading Simulation INFO Done"<<endl;


		   /* Now print Info */
			 LogFile<<"-------------------------------------------------------------------"<<endl;
			 LogFile<<"                      Simulation Information                       "<<endl;
			 LogFile<<"-------------------------------------------------------------------"<<endl;
			 LogFile<<"                       Particle information                        "<<endl;

			 if(m_Sim_ParticleType==1)
			 {
		       for( int i=0;i<m_num_ParticleObjects;i++)
		       {
			     LogFile<<"-------------------------------------------------------------------"<<endl;
			     LogFile<<"                      Particle "<<i<<endl;
			     Print_PObject(&m_ParticleObjectsList[i]);
			     LogFile<<"-------------------------------------------------------------------\n"<<endl;

		        }
			 }
		   LogFile<<""<<endl;



		LogFile<<""<<endl;
		LogFile<<"                      End of Data Read                           "<<endl;
		LogFile<<"-------------------------------------------------------------------"<<endl;
		LogFile<<""<<endl;

	  WorldF.close();
  }/* End World*/




 LogFile<<"INFO: FileIO Complete "<<endl;

 LogFile.close();

}


/*------*/
void KSimulationData::ReadWObject(int i)
{
  string dumS;
  int    dumI;

            WObjectsF>>dumS>>m_WorldObjectDescp.name[i];
			WObjectsF>>dumS>>m_WorldObjectDescp.descp[i];
			WObjectsF>>dumS;


			WObjectsF>>dumS>>m_WorldObjectsList[i].num_vertex;

			for( int j=0; j<m_WorldObjectsList[i].num_vertex;j++ )
			{
				WObjectsF>>dumI>>m_WorldObjectsList[i].vertex[j].x
				               >>m_WorldObjectsList[i].vertex[j].y
				               >>m_WorldObjectsList[i].vertex[j].z;

				/* Translates from the origin */
//				if(m_WorldObjectsList[i].is_moving)
//				{
//					m_WorldObjectsList[i].vertex[j].x += m_WorldObjectsList[i].cyl_geo.radius;
//				}

			}

			WObjectsF>>dumS>>m_WorldObjectsList[i].num_surfaces;

			for( int j=0; j<m_WorldObjectsList[i].num_surfaces;j++ )
			{
				float3 sum=zeroF();

				WObjectsF>>dumI>>dumS;

				if(dumS[1]=='S')
				{
				 m_OpenGLObject.world_surface_Shading[i][j] = true;

				}
				else
				{
					m_OpenGLObject.world_surface_Shading[i][j] = false;
				}

				if(dumI==-1)
				{
					m_WorldObjectsList[i].surface_type= cylinder;
				}
				else
				{
					m_WorldObjectsList[i].surface_type = plane;

				}

				if( m_WorldObjectsList[i].surface_type == plane )
				{

                 /* Read Tol for calculation */
				 WObjectsF>>dumS>>m_WorldObjectsList[i].surfaces[j].Tol;

				  WObjectsF>>dumS>>m_WorldObjectsList[i].surfaces[j].num_vertex;

                   for( int f=0; f < m_WorldObjectsList[i].surfaces[j].num_vertex; f++ )
			       {
			    	 WObjectsF>>m_WorldObjectsList[i].surfaces[j].vertex_Order[f];

			    	 m_WorldObjectsList[i].surfaces[j].vertex_Order[f] =
			    	 		m_WorldObjectsList[i].surfaces[j].vertex_Order[f]-1;

			    	  sum = sum + m_WorldObjectsList[i].vertex[
			    	             m_WorldObjectsList[i].surfaces[j].vertex_Order[f]];

			       }
				 }
				 else
				 {
					 WObjectsF>>dumS>>m_WorldObjectsList[i].cyl_geo.radius;

					 m_WorldObjectsList[i].cyl_geo.radius*=0.50f;
					 WObjectsF>>dumS>>m_WorldObjectsList[i].cyl_geo.height;
					 string boolval="";
					 WObjectsF>>dumS>>boolval;

					 if( boolval=="true" )
					 {
					   m_WorldObjectsList[i].cyl_geo.has_top_cap =true;
					 }
					 else
					 {
					   m_WorldObjectsList[i].cyl_geo.has_top_cap =false;
					 }

					 boolval="";
				     WObjectsF>>dumS>>boolval;

				     if( boolval=="true" )
					 {
					   m_WorldObjectsList[i].cyl_geo.has_bot_cap =true;
					 }
					 else
					 {
					   m_WorldObjectsList[i].cyl_geo.has_bot_cap =false;
					 }

				     /* Get TOP  */
				     WObjectsF>>dumS>> m_WorldObjectsList[i].cyl_geo.normal_top_cap.x
				                    >> m_WorldObjectsList[i].cyl_geo.normal_top_cap.y
				                    >> m_WorldObjectsList[i].cyl_geo.normal_top_cap.z;

				     /* Get BOT  */
				     WObjectsF>>dumS>> m_WorldObjectsList[i].cyl_geo.normal_bot_cap.x
				                    >> m_WorldObjectsList[i].cyl_geo.normal_bot_cap.y
				                    >> m_WorldObjectsList[i].cyl_geo.normal_bot_cap.z;


				     float3 offset;
				     WObjectsF>>dumS>>offset.x>>offset.y>>offset.z;
				     m_WorldObjectsList[i].cyl_geo.AXIS = make_float3(1.0f,1.0f,1.0f) - m_WorldObjectsList[i].cyl_geo.normal_bot_cap;

				     LogFile<<"Cylinder Axis:"<<m_WorldObjectsList[i].cyl_geo.AXIS.x<<
				    		               m_WorldObjectsList[i].cyl_geo.AXIS.y<<
				    		               m_WorldObjectsList[i].cyl_geo.AXIS.z<<endl;

				    m_WorldObjectsList[i].cyl_geo.center_bot_cap = offset +
				       m_WorldObjectsList[i].cyl_geo.radius*(make_float3(1.0f,1.0f,1.0f)-m_WorldObjectsList[i].cyl_geo.normal_bot_cap);

				     m_WorldObjectsList[i].cyl_geo.center_top_cap = m_WorldObjectsList[i].cyl_geo.center_bot_cap +
				    		 m_WorldObjectsList[i].cyl_geo.normal_bot_cap*m_WorldObjectsList[i].cyl_geo.height;


				 }


				 /* Calculate some parms */
				if( m_WorldObjectsList[i].surface_type == plane )
				{

				   if( m_WorldObjectsList[i].surfaces[j].num_vertex==3 )
				   {

				     float3  V1 = m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[2]] -
				         		  m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[0]];

				     float3  V2 = m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[1]] -
				   	  			  m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[2]];

		             float3 n   = cross(V2,V1);
		             m_WorldObjectsList[i].surfaces[j].normal = n/length(n);
		             m_WorldObjectsList[i].surfaces[j].area = 0.5f*length(n);

			        }
			        else /* Primitive Type 2: Quad */
			        if( m_WorldObjectsList[i].surfaces[j].num_vertex==4 )
			        {
			    	   float3  V1 = m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[1]] -
			    	 				m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[0]];

			           float3  V2 = m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[2]] -
			    			        m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[1]];


			           float3  V3 = m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[3]] -
			        				    		     m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[2]];

			            float3 V4 = m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[0]] -
			        			     m_WorldObjectsList[i].vertex[m_WorldObjectsList[i].surfaces[j].vertex_Order[3]];

					    float3 n         = cross(V2,V1);
					    m_WorldObjectsList[i].surfaces[j].normal = n/length(n);
					    m_WorldObjectsList[i].surfaces[j].area = 0.5f*length(n) + 0.5f*length(cross(V3,V4)) ;
				     }
				     m_WorldObjectsList[i].surfaces[j].centroid = (1.0f/m_WorldObjectsList[i].surfaces[j].num_vertex)*sum;

				}

			}/* End reading surfaces */



}



void KSimulationData::Set_SpringConstantsParticle_TC(int i,int j, double COR_i,double COR_j)
{
	double COR = 0.50f*(COR_i+COR_j);
	double m   = 1.0/( (1.0/m_ParticleObjectsList[i].mass) + (1.0/m_ParticleObjectsList[j].mass) );


	if( COR>=0.30 && COR<=0.90)
	{
	  COR-=0.038080;
	}


	/* If its the same particle */
	if( i==j )
	{
	   m = m_ParticleObjectsList[i].mass;
	}

	double Pi=2.0*asin(1.0);


	/* PP[0] is the value read from file */

	m_ParticleObjectsList[i].PP[j].Cn = 2.0*m*(-log(COR)/(tc_PP));
	m_ParticleObjectsList[i].PP[j].Kn = (m/(tc_PP*tc_PP))*(log(COR)*log(COR) + Pi*Pi);

	m_ParticleObjectsList[i].PP[j].Fric_static  = 0.50*(m_ParticleObjectsList[i].PP[0].Fric_static  + m_ParticleObjectsList[j].PP[0].Fric_static);
	m_ParticleObjectsList[i].PP[j].Fric_kinetic = 0.50*(m_ParticleObjectsList[i].PP[0].Fric_kinetic + m_ParticleObjectsList[j].PP[0].Fric_kinetic);


}


void KSimulationData::Set_SpringConstantsSurface_TC(int i, double COR)
{
	double m = m_ParticleObjectsList[i].mass;

	double Pi = 2.0*asin(1.0);

	if( COR>=0.30 && COR<=0.90)
	{
	  COR-=0.038080;
	}

	m_ParticleObjectsList[i].surface_Cn = 2.0*m*(-log(COR)/(tc_PS));
	m_ParticleObjectsList[i].surface_Kn =(m/(tc_PS*tc_PS))*(log(COR)*log(COR) + Pi*Pi);

}

void KSimulationData::Set_SpringConstantsDSurface_TC(int i, double COR)
{
	double m  = m_ParticleObjectsList[i].mass;
	double Pi = 2.0*asin(1.0);

	if( COR>=0.30 && COR<=0.90)
	{
	  COR-=0.038080;
	}

	m_ParticleObjectsList[i].Dsurface_Cn = 2.0*m*(-log(COR)/(tc_PD));
	m_ParticleObjectsList[i].Dsurface_Kn =(m/(tc_PD*tc_PD))*(log(COR)*log(COR) + Pi*Pi);

}




/* Sets parameters when the user specfiys spring stiffness */
void KSimulationData::Set_SpringConstantsParticleKN(int i,int j, double COR_i,double COR_j)
{
	double COR = COR_i;
	double m = 1.0/( (1.0/m_ParticleObjectsList[i].mass) + (1.0/m_ParticleObjectsList[i].mass) );


	double Pi=2.0*asin(1.0);

	if(i==j)
    {

	  if( COR < 1.000 )
      {
			if( COR>=0.30 && COR<=0.90)
			{
			  COR-=0.038080;
			}

	   m_ParticleObjectsList[i].PP[j].Cn = -2.0*log(COR)*sqrt(m_ParticleObjectsList[i].PP[j].Kn*m)/(sqrt((log(COR)*log(COR) + Pi*Pi)));

      }
	  else
	  {
		m_ParticleObjectsList[i].PP[j].Cn = 0.0f;
	  }
    }

	  /* Average properties */
      if(i!=j)
	  {

    	m = 1.0/( (1.0/m_ParticleObjectsList[i].mass) + (1.0/m_ParticleObjectsList[j].mass) );


    	//printf(" %d and %d  mass %f",m,i,j);

        COR = 0.50f*(COR_i+COR_j);

    	if( COR>=0.30 && COR<=0.90)
    	{
    	  COR-=0.038080;
    	}


        m_ParticleObjectsList[i].PP[j].Kn   = 0.50f*(m_ParticleObjectsList[i].PP[i].Kn+m_ParticleObjectsList[j].PP[j].Kn);
	    m_ParticleObjectsList[i].PP[j].Fric_static = 0.50*(m_ParticleObjectsList[i].PP[i].Fric_static + m_ParticleObjectsList[j].PP[j].Fric_static);
	    m_ParticleObjectsList[i].PP[j].Fric_kinetic = 0.50*(m_ParticleObjectsList[i].PP[i].Fric_kinetic + m_ParticleObjectsList[j].PP[j].Fric_kinetic);

        if(COR<1.000)
        {
	      m_ParticleObjectsList[i].PP[j].Cn = -2.0*log(COR)*sqrt(m_ParticleObjectsList[i].PP[j].Kn*m)/(sqrt((log(COR)*log(COR) + Pi*Pi)));

        }
	    else
	    {
		  m_ParticleObjectsList[i].PP[j].Cn = 0.0f;
	    }
	}



}


void KSimulationData::Set_SpringConstantsSurface_KN(int i, double COR)
{
	double m = m_ParticleObjectsList[i].mass;





	double Pi=2.0*asin(1.0);

   if(COR<1.000)
   {
		if( COR>=0.30 && COR<=0.90)
		{
		  COR-=0.038080;
		}

	 m_ParticleObjectsList[i].surface_Cn = -2.0*log(COR)*sqrt(m_ParticleObjectsList[i].surface_Kn*m)/(sqrt((log(COR)*log(COR) + Pi*Pi)));

	 //m_ParticleObjectsList[i].surface_Ct=0.0f;
   }
   else
   {
	   m_ParticleObjectsList[i].surface_Cn=0.0f;
	   //m_ParticleObjectsList[i].surface_Ct=0.0f;
   }

}


void KSimulationData::Set_SpringConstantsDSurfaceKN(int i, double COR)
{
	double m = m_ParticleObjectsList[i].mass;



	double Pi=2.0*asin(1.0);

	if(COR<1.000)
	{
		if( COR>=0.30 && COR<=0.90)
		{
		  COR-=0.038080;
		}

	   m_ParticleObjectsList[i].Dsurface_Cn = -2.0*log(COR)*sqrt(m_ParticleObjectsList[i].Dsurface_Kn*m)/(sqrt((log(COR)*log(COR) + Pi*Pi)));

	}
	else
	{
		m_ParticleObjectsList[i].Dsurface_Cn = 0.0f;
	}
}




void KSimulationData::ReadPObject_Sphere(int i)
{
    string dumS;
    int    dumI;


    PObjectsF>>dumS>>m_ParticleObjectDescp.name[i];
    PObjectsF>>dumS>>m_ParticleObjectDescp.descp[i];
    PObjectsF>>dumS;

    	LogFile<<" Info : Sphere Particle"<<endl;

    	double den;
    	double COR_S;
    	double COR_D;

    	PObjectsF>>dumS>>m_ParticleObjectsList[i].radius;

    	m_ParticleObjectsList[i].radius*=0.500;
    	PObjectsF>>dumS>>den;
    	m_ParticleObjectsList[i].mass = (4.0/3.0)*2.0*asin(1.0)*pow(m_ParticleObjectsList[i].radius,3.0)*den*1E-6;

    	//m_ParticleObjectsList[i].mass =1.0f;

        printf("Volume(cm3) %f  mass(kg) %f \n",((4.0/3.0)*2.0*asin(1.0)*pow(m_ParticleObjectsList[i].radius,3.0) ), m_ParticleObjectsList[i].mass);

        if(m_ForceSpec==0)
        {

        	float dumF;
			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR[i];
			PObjectsF>>dumS>>tc_PP;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[0].Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[0].Fric_kinetic;

			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_S;
			PObjectsF>>dumS>>tc_PS;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_kinetic;


			tc_PS*=m_delta_t;

			tc_PP*=m_delta_t;



			Set_SpringConstantsSurface_TC(i,COR_S);

			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_D;
			PObjectsF>>dumS>>tc_PD;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_kinetic;


			tc_PD*=m_delta_t;

    	    Set_SpringConstantsDSurface_TC(i,COR_D);
        }


        if(m_ForceSpec==1)
        {
			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR[i];
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[i].Kn;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[i].Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[i].Fric_kinetic;

			//cout<<"  Kn:    "<<i<<" "<<m_ParticleObjectsList[i].PP[i].Kn <<std::endl;


			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_S;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Kn;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_kinetic;



			Set_SpringConstantsSurface_KN(i,COR_S);

			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_D;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Kn;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_kinetic;


    	    Set_SpringConstantsDSurfaceKN(i,COR_D);
        }

    	LogFile<<" radius: "<<m_ParticleObjectsList[i].radius<<endl;
    	LogFile<<" mass:"<<m_ParticleObjectsList[i].mass<<endl;

    	LogFile<<""<<endl;



}



/*---------------------------------------------------------------------*/
                   /* Makes the Local Particle Info */
/*---------------------------------------------------------------------------*/
void KSimulationData:: Make_Particle_LocalInfo(POBJ * OBJ)
{

	/* Make the vectors from the CM to the vertcies*/
	float3 vec;
	float3 Max = make_float3( 0.00f, 0.00f, 0.00f );
	float3 Min = make_float3( 10.0f, 10.0f, 10.0f );
	float3 test;
	int extreme_vetex_index=0;
    float dis=0.0f;

	for ( int i=0; i<OBJ->num_vertex;i++ )
	{
		OBJ->Local_Vertex[i]= OBJ->vertex[i]- OBJ->COM;
		//OBJ->Local_Vertex_Dis [i]= length(OBJ->Local_Vertex[i]);

	  /* Now get the maximum bound in each direction */
	  test = OBJ->Local_Vertex[i];//*Particle1.Local_Vertex_Dis[i];



	 if(length(OBJ->Local_Vertex[i])>dis)
	 {
		dis = length(OBJ->Local_Vertex[i]);
		extreme_vetex_index = i;
	 }
  }/* End loop */

    OBJ->radius = dis;

}



void KSimulationData::ReadPObject_Poly(int i)
{
    string dumS;
    int    dumI;


   /* Debug Info */
    Quaterion RotQ;

    RotQ = make_quaterion(45.0f,0.0f,0.0f,1.0f);

   // printf(" Quart %f %f %f %f \n",RotQ.w,RotQ.x,RotQ.y,RotQ.z);


    if(i==0)
    {
     RotQ = make_quaterion(45.0f,0.0f,0.0f,1.0f);
    }

    if(i==4)
    {
     RotQ = make_quaterion(-90.0f,0.0f,0.0f,1.0f);
    }

    if(i==5)
    {
     RotQ = make_quaterion(90.0f,1.0f,0.0f,0.0f);
    }

    if(i==6)
    {
     RotQ = make_quaterion(-90.0f,1.0f,0.0f,0.0f);
    }


    Quaterion RotQI = conjugate(RotQ);
   /* End Debug Info */

    int Format;

    PObjectsF>>dumS>>m_ParticleObjectDescp.name[i];
    PObjectsF>>dumS>>m_ParticleObjectDescp.descp[i];
    PObjectsF>>dumS>>Format;

    printf(" Format %d \n",Format);

    float3 Offset;

    if(Format==1)
    {
      PObjectsF>>dumS>>Offset.x>>Offset.y>>Offset.z;
    }

    /* Read Geometrical Data */
    PObjectsF>>dumS;
    PObjectsF>>dumS;
    PObjectsF>>dumS;

	PObjectsF>>dumS>>m_ParticleObjectsList[i].num_vertex;



	m_InitPosGrid.p_size[i].x=0.0f;
	m_InitPosGrid.p_size[i].y=0.0f;
	m_InitPosGrid.p_size[i].z=0.0f;

	float3 sum=make_float3(0.0f,0.0f,0.0f);




	   for ( int k=0; k<m_ParticleObjectsList[i].num_vertex; k++ )
	   {

	     if(Format==0)
		 {
		  PObjectsF>>dumI>>m_ParticleObjectsList[i].vertex[k].x
		               >>m_ParticleObjectsList[i].vertex[k].y
		               >>m_ParticleObjectsList[i].vertex[k].z;
		 }
	     else
	     {
			  PObjectsF>>m_ParticleObjectsList[i].vertex[k].x
			               >>m_ParticleObjectsList[i].vertex[k].y
			               >>m_ParticleObjectsList[i].vertex[k].z;

			  m_ParticleObjectsList[i].vertex[k]+=Offset;
	     }

	     //printf( " %f  %f  %f ; \n",m_ParticleObjectsList[i].vertex[k].x,m_ParticleObjectsList[i].vertex[k].y,m_ParticleObjectsList[i].vertex[k].z);

		  /* Get bounds */
			if (m_ParticleObjectsList[i].vertex[k].x> m_InitPosGrid.p_size[i].x)
			{
				m_InitPosGrid.p_size[i].x = m_ParticleObjectsList[i].vertex[k].x;
			}

			if (m_ParticleObjectsList[i].vertex[k].y> m_InitPosGrid.p_size[i].y)
			{
				m_InitPosGrid.p_size[i].y = m_ParticleObjectsList[i].vertex[k].y;
			}

			if (m_ParticleObjectsList[i].vertex[k].z> m_InitPosGrid.p_size[i].z)
			{
				m_InitPosGrid.p_size[i].z = m_ParticleObjectsList[i].vertex[k].z;
			}

		  sum+=m_ParticleObjectsList[i].vertex[k];

       }

	   /* Calculate COM */
	   m_ParticleObjectsList[i].COM = sum*(1.0f/(m_ParticleObjectsList[i].num_vertex));

	   /* NICO ADD Inertia */
   	float SurArea=0.0f;
   	float Vol=0.0f;

	  // printf("reading faces\n");

	   PObjectsF>>dumS>>m_ParticleObjectsList[i].num_faces;

	   /* Read Faces */
       float3 COM = zeroF();

       char delim[1];
       float TotArea;
	   for( int j=0; j<m_ParticleObjectsList[i].num_faces;j++ )
	   {

	      float3 sum = zeroF();

		  if(Format==0)
		  {
	        PObjectsF>>dumI>>dumS>>m_ParticleObjectsList[i].face[j].num_vertex;
		  }
		  else
		  {
			  PObjectsF>>m_ParticleObjectsList[i].face[j].num_vertex;
			   //printf("reading face %d with %d Vertex \n",j,m_ParticleObjectsList[i].face[j].num_vertex);

		  }

           /* Read the vertex order for each face */
	      for( int f=0; f < m_ParticleObjectsList[i].face[j].num_vertex; f++ )
	      {
	    	 PObjectsF>>m_ParticleObjectsList[i].face[j].vertex_Order[f];

			  if(Format==1 && f<(m_ParticleObjectsList[i].face[j].num_vertex-1))
			  {
				  PObjectsF>>delim;
			  }



	    	 m_ParticleObjectsList[i].face[j].vertex_Order[f] =
	    	           m_ParticleObjectsList[i].face[j].vertex_Order[f]-1;

	    	 sum = sum + m_ParticleObjectsList[i].vertex[
	    		         m_ParticleObjectsList[i].face[j].vertex_Order[f]];
           }




           m_ParticleObjectsList[i].face[j].centroid = (1.0f/m_ParticleObjectsList[i].face[j].num_vertex)*sum;

           if( m_ParticleObjectsList[i].face[j].num_vertex==3 )
           {

              float3  V1 = m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[1]] -
         		      m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[2]];

              float3  V2 = m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[2]] -
        		      m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[0]];



              float3 n   = cross(V1,V2);

              m_ParticleObjectsList[i].face[j].normal = n/length(n);
              m_ParticleObjectsList[i].face[j].area   = 0.5f*length(n);

              /* Volume of a tetra */
              float3 AD= (m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[0]]-m_ParticleObjectsList[i].COM);
              float3 BD= (m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[1]]-m_ParticleObjectsList[i].COM);
              float3 CD= (m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[2]]-m_ParticleObjectsList[i].COM);

              float VolTet= sqrt(dot(AD,cross(BD,CD)))/6.0000f;

              Vol+=VolTet;

            }
            else
            {
            	int nv=m_ParticleObjectsList[i].face[j].num_vertex;

            	TotArea=0.0f;

                float3  V1 = m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[2]] -
         		      m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[3]];

                float3  V2 = m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[3]] -
        		      m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[0]];

                float3 n         = cross(V1,V2);

                m_ParticleObjectsList[i].face[j].normal = n/length(n);

                /* Area Calc */

            	V1= m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[nv-1]] -
            			m_ParticleObjectsList[i].face[j].centroid;

            	V2= m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[0]] -
            			m_ParticleObjectsList[i].face[j].centroid;

                TotArea=0.50f*length(cross(V1,V2));

                /* Volume of a tetra */
                float3 AD= (m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[nv-1]]-m_ParticleObjectsList[i].COM);
                float3 BD= (m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[0]]-m_ParticleObjectsList[i].COM);
                float3 CD= (m_ParticleObjectsList[i].face[j].centroid-m_ParticleObjectsList[i].COM);

                float VolTet= dot(AD,cross(BD,CD))/6.0000f;

              	Vol+=VolTet;



              for(int tr=0;tr<nv-1;tr++)

              {
            	V1= m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[tr]] -
            			m_ParticleObjectsList[i].face[j].centroid;

            	V2= m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[tr+1]] -
            			m_ParticleObjectsList[i].face[j].centroid;

            	AD = (m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[tr]]-m_ParticleObjectsList[i].COM);
            	BD = (m_ParticleObjectsList[i].vertex[m_ParticleObjectsList[i].face[j].vertex_Order[tr+1]]-m_ParticleObjectsList[i].COM);
            	CD = (m_ParticleObjectsList[i].face[j].centroid-m_ParticleObjectsList[i].COM);


            	/* NICO WORK HERE */
            	VolTet= dot(AD,cross(BD,CD))/6.0000f;

            	Vol+=VolTet;


            	TotArea += 0.50f*length(cross(V1,V2));
              }

              SurArea+=TotArea;
            }

           m_ParticleObjectsList[i].face[j].area = TotArea;

	   }/* End reading faces */



	   /* Read Edges */



	      PObjectsF>>dumS>>m_ParticleObjectsList[i].num_edges;

	      printf("Reading %d Edges \n",m_ParticleObjectsList[i].num_edges);

	      for (int k=0;k<m_ParticleObjectsList[i].num_edges;k++)
	      {

		    	  PObjectsF>>m_ParticleObjectsList[i].edge[k].point_vindex[0]
		    	           >>m_ParticleObjectsList[i].edge[k].point_vindex[1];

	    	  m_ParticleObjectsList[i].edge[k].point_vindex[0]-=1;
	    	  m_ParticleObjectsList[i].edge[k].point_vindex[1]-=1;

	      }

	      printf("Read Edges\n");

	      /* Read Polyhedral Data */
	      PObjectsF>>dumS;
	      PObjectsF>>dumS;
	      PObjectsF>>dumS;


	   float density;

	   PObjectsF>>dumS>>density;
	   m_ParticleObjectsList[i].mass=density*Vol*1E-6;


	  // m_ParticleObjectsList[i].mass = 0.089457f;

	   printf("Done Faces surface area(cm2) %f Volume(cm3) %f Mass(kg) %f \n",SurArea,Vol,m_ParticleObjectsList[i].mass);

	   //printf("!mass fixed at 1\n");
	   //m_ParticleObjectsList[i].mass=0.10;

	   PObjectsF>>dumS;

  		   PObjectsF>>m_ParticleObjectsList[i].InertiaT[0]>>m_ParticleObjectsList[i].InertiaT[1]>>m_ParticleObjectsList[i].InertiaT[2]
                    >>m_ParticleObjectsList[i].InertiaT[3]>>m_ParticleObjectsList[i].InertiaT[4]>>m_ParticleObjectsList[i].InertiaT[5]
                    >>m_ParticleObjectsList[i].InertiaT[6]>>m_ParticleObjectsList[i].InertiaT[7]>>m_ParticleObjectsList[i].InertiaT[8];

  		   /* Invert inertia tensor */
  		   for (int j=0;j<9;j++)
  		   {
  			 if (m_ParticleObjectsList[i].InertiaT[j]>0.0f)
  			 {
  			  m_ParticleObjectsList[i].InertiaT[j]=1.0/m_ParticleObjectsList[i].InertiaT[j];
  			 }
  		   }

  	   Make_Particle_LocalInfo(&m_ParticleObjectsList[i]);
       LogFile<<" B_Radius: "<<m_ParticleObjectsList[i].radius<<endl;
       cout<<" B_Radius: "<<m_ParticleObjectsList[i].radius<<" Pen Limit: "<<m_ParticleObjectsList[i].radius*0.050<<endl;
       LogFile<<""<<endl;



       LogFile<<" Info : Reading Spring Constants "<<endl;

	      /* Read Polyhedral Data */
	      PObjectsF>>dumS;
	      PObjectsF>>dumS;
	      PObjectsF>>dumS;

		double COR_S;
		double COR_D;


		   if(m_ForceSpec==0)
		   {
			float dumF;
			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR[i];
			PObjectsF>>dumS>>tc_PP;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[0].Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[0].Fric_kinetic;

			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_S;
			PObjectsF>>dumS>>tc_PS;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_kinetic;

			tc_PS*=m_delta_t;

			tc_PP*=m_delta_t;



			Set_SpringConstantsSurface_TC(i,COR_S);

			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_D;
			PObjectsF>>dumS>>tc_PD;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_kinetic;


			tc_PD*=m_delta_t;

			Set_SpringConstantsDSurface_TC(i,COR_D);
		   }


		   if(m_ForceSpec==1)
		   {
			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR[i];
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[i].Kn;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[i].Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].PP[i].Fric_kinetic;


			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_S;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Kn;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].surface_Fric_kinetic;



			Set_SpringConstantsSurface_KN(i,COR_S);

			PObjectsF>>dumS;
			PObjectsF>>dumS>>COR_D;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Kn;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_static;
			PObjectsF>>dumS>>m_ParticleObjectsList[i].Dsurface_Fric_kinetic;


			Set_SpringConstantsDSurfaceKN(i,COR_D);
		   }


           	LogFile<<""<<endl;



}



void KSimulationData::Print_PObject(POBJ * OBJ)
{
  LogFile<<""<<endl;
  LogFile<<" VERTEX: "<<OBJ->num_vertex<<endl;
  LogFile<<" FACES: "<<OBJ->num_faces<<endl;
  LogFile<<" BoundRadius: "<<OBJ->radius<<endl;

  LogFile<<""<<endl;

  LogFile<<"COM : "<<OBJ->COM.x<<", "<<OBJ->COM.y<<", "<<OBJ->COM.z<<endl;
  LogFile<<" MASS(Kg)"<<OBJ->mass<<endl;
  LogFile<<""<<endl;
  LogFile<<"                Inertia Tensor "<<endl;
  LogFile<< OBJ->InertiaT[0]<<", "<< OBJ->InertiaT[1]<<", "<< OBJ->InertiaT[2]<<endl;
  LogFile<< OBJ->InertiaT[3]<<", "<< OBJ->InertiaT[4]<<", "<< OBJ->InertiaT[5]<<endl;
  LogFile<< OBJ->InertiaT[6]<<", "<< OBJ->InertiaT[7]<<", "<< OBJ->InertiaT[8]<<endl;

  LogFile<<""<<endl;

}


void KSimulationData::ReadDObject(int i)
{
    string dumS;
    int    dumI;




    DObjectsF>>dumS>>dumS;
    DObjectsF>>dumS>>dumS;

    DObjectsF>>dumS>>m_DynamicObjectsList[i].btype;


    /* If its a cylinder */
    if(m_DynamicObjectsList[i].btype==1)
    {
      DObjectsF>>dumS>> m_DynamicObjectsList[i].Axis.x>>
	  				     m_DynamicObjectsList[i].Axis.y>>
    				     m_DynamicObjectsList[i].Axis.z;
    }

	DObjectsF>>dumS>> m_DynamicObjectsList[i].COM.x>>
					   m_DynamicObjectsList[i].COM.y>>
					   m_DynamicObjectsList[i].COM.z;


    DObjectsF>>dumS>>m_DynamicObjectsList[i].boundR;


    DObjectsF>>dumS>>m_DynamicObjectsList[i].num_vertex;

	/* Dynamic Objects */
    if( m_DynamicObjectsList[i].num_vertex > 0 )
    {

	   for ( int k=0; k<m_DynamicObjectsList[i].num_vertex; k++ )
	   {

		  DObjectsF>>dumI>>m_DynamicObjectsList[i].vertex[k].x
		               >>m_DynamicObjectsList[i].vertex[k].y
		               >>m_DynamicObjectsList[i].vertex[k].z;

       }

	   DObjectsF>>dumS>>m_DynamicObjectsList[i].num_faces;

	   /* Read Faces */
       float3 COM = zeroF();

	   for( int j=0; j<m_DynamicObjectsList[i].num_faces;j++ )
	   {
	      float3 sum = zeroF();

	      DObjectsF>>dumI>>dumS>>dumS>>m_DynamicObjectsList[i].faces[j].Tol;
	      DObjectsF>>dumS>>m_DynamicObjectsList[i].faces[j].num_vertex;

           /* Read the vertex order for each face */
	      for( int f=0; f < m_DynamicObjectsList[i].faces[j].num_vertex; f++ )
	      {
	    	 DObjectsF>>m_DynamicObjectsList[i].faces[j].vertex_Order[f];

	    	 m_DynamicObjectsList[i].faces[j].vertex_Order[f] =
	    	           m_DynamicObjectsList[i].faces[j].vertex_Order[f]-1;

	    	 sum = sum + m_DynamicObjectsList[i].vertex[
	    		         m_DynamicObjectsList[i].faces[j].vertex_Order[f]];
           }

           m_DynamicObjectsList[i].faces[j].centroid = (1.0f/m_DynamicObjectsList[i].faces[j].num_vertex)*sum;

           if( m_DynamicObjectsList[i].faces[j].num_vertex==3 )
           {

              float3  V1 = m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[2]] -
        		      m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[0]];

              float3  V2 = m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[1]] -
        		      m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[2]];

              float3 n   = cross(V1,V2);

              m_DynamicObjectsList[i].faces[j].normal = n/length(n);
              m_DynamicObjectsList[i].faces[j].area = 0.5f*length(n);

            }
            /* Primitive Type 2: Quad */
            else if( m_DynamicObjectsList[i].faces[j].num_vertex==4 )
            {
               float3  V1 = m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[1]] -
        		      m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[0]];

               float3  V2 = m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[2]] -
        		      m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[1]];


               float3  V3 = m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[3]] -
        		      m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[2]];


               float3  V4 = m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[0]] -
          		      m_DynamicObjectsList[i].vertex[m_DynamicObjectsList[i].faces[j].vertex_Order[3]];


               float3 n         = cross(V2,V1);

               m_DynamicObjectsList[i].faces[j].normal = n/length(n);
               m_DynamicObjectsList[i].faces[j].area = 0.5f*length(n) + 0.5f*length(cross(V3,V4)) ;

            }

	   }/* End reading faces */

	   DObjectsF>>dumS>>m_DynamicObjectsList[i].num_edges;

	      for (int k=0;k<m_DynamicObjectsList[i].num_edges;k++)
	      {
	    	  DObjectsF>>m_DynamicObjectsList[i].edge[k].point_vindex[0]
	    	            >>m_DynamicObjectsList[i].edge[k].point_vindex[1];

	    	  m_DynamicObjectsList[i].edge[k].point_vindex[0]-=1;
	    	  m_DynamicObjectsList[i].edge[k].point_vindex[1]-=1;

	      }

       Make_DObject_LocalInfo(&m_DynamicObjectsList[i],i);

    }/* End Polyhedra Read */


}

/*---------------------------------------------------------------------------*/
                          /* Destructor */
/*---------------------------------------------------------------------------*/
KSimulationData::~KSimulationData()
{

}



/*---------------------------------------------------------------------*/
                   /* Makes the Local Particle Info */
/*---------------------------------------------------------------------------*/
void KSimulationData:: Make_DObject_LocalInfo ( DOBJ * OBJ, int index )
{

	/* Make the vectors from the CM to the vertcies*/
	float3 vec;
	float3 Max = make_float3( 0.00f, 0.00f, 0.00f );
	float3 Min = make_float3( 10.0f, 10.0f, 10.0f );
	float3 test;
	int extreme_vetex_index=0;
    float dis=0.0f;


	for ( int i=0; i<OBJ->num_vertex;i++ )
	{
		m_KDynamicObject_HostList[index].Local_Vertex[i]= OBJ->vertex[i]- OBJ->COM;

  }/* End loop */


}


void KSimulationData::Print_WObject(WOBJ * OBJ)
{
  LogFile<<" "<<endl;
  //LogFile<<" WObject: "<<OBJ->name<<endl;
 // LogFile<<OBJ->desc<<endl;
  LogFile<<" "<<endl;
  for (int k=0; k<OBJ->num_vertex; k++)
  {
	  LogFile<<" vertex "<<k<<" = ";
	  PrintFloat3( OBJ->vertex[k] );
  }
  LogFile<<" "<<endl;
  LogFile<<" Number Surfaces = "<<OBJ->num_surfaces<<endl;

  for ( int i=0; i<OBJ->num_surfaces; i++)
  {
	  LogFile<<" Surface "<<i<<endl;
	//  LogFile<< OBJ->surfaces[i].name<<endl;
	  LogFile<<" "<<endl;

	  if(OBJ->surface_type==plane)
      {
	    LogFile<<" Number Vertex = "<<OBJ->surfaces[i].num_vertex<<endl;

	    /* Loop over all vertex per surface */
	    for (int k=0; k<OBJ->surfaces[i].num_vertex; k++)
	    {
		   LogFile<<" "<<OBJ->surfaces[i].vertex_Order[k]<<" ";
	    }
	    LogFile<<" "<<endl;
	    LogFile<<" centroid  = ";
	    PrintFloat3(OBJ->surfaces[i].centroid);

	    LogFile<<" normal  = ";
	    PrintFloat3(OBJ->surfaces[i].normal);
	    LogFile<<" "<<endl;
      }
	  else
	  {
		  LogFile<<" Radius = "<<OBJ->cyl_geo.radius<<endl;
		  LogFile<<" Height = "<<OBJ->cyl_geo.height<<endl;
		  LogFile<<" Bottom : "<<OBJ->cyl_geo.has_bot_cap<<endl;
		  if( OBJ->cyl_geo.has_bot_cap)
		  {
			LogFile<<" center  = ";
			PrintFloat3(OBJ->cyl_geo.center_bot_cap);

			LogFile<<" normal  = ";
			PrintFloat3(OBJ->cyl_geo.normal_bot_cap);
		  }
		  LogFile<<" "<<endl;

		  LogFile<<" Top : "<<OBJ->cyl_geo.has_top_cap<<endl;
		  if( OBJ->cyl_geo.has_top_cap)
		  {
			LogFile<<" center  = ";
			PrintFloat3(OBJ->cyl_geo.center_top_cap);

			LogFile<<" normal  = ";
			PrintFloat3(OBJ->cyl_geo.normal_top_cap);
		  }
	  }

	  LogFile<<" "<<endl;

  }


}


/*---------------------------------------------------------------------------*/
/*                      Makes a new Dynamic Object                           */
/*---------------------------------------------------------------------------*/

void KSimulationData::Make_New_Lifter(float3 about,int index, int d_index, float angle)
{

	float3 sum = make_float3(0.0f,0.0f,0.0f);

	m_DynamicObjectsList[d_index] = m_DynamicObjectsList[index];

     /* Rotate Vertex */
     for(int k=0;k<m_DynamicObjectsList[d_index].num_vertex;k++)
     {
    	 float3 point = m_DynamicObjectsList[d_index].vertex[k] - about ;

    	 m_DynamicObjectsList[d_index].vertex[k].x = cos(angle)*point.x + sin(angle)*point.y;
    	 m_DynamicObjectsList[d_index].vertex[k].y = -sin(angle)*point.x + cos(angle)*point.y;
    	 m_DynamicObjectsList[d_index].vertex[k]   = m_DynamicObjectsList[d_index].vertex[k] + about;

     }

     /* Update Surfaces */
     for(int j=0;j<m_DynamicObjectsList[d_index].num_faces;j++)
     {
    	 sum = make_float3(0.0f,0.0f,0.0f);

         for(int f=0;f<m_DynamicObjectsList[d_index].faces[j].num_vertex;f++)
         {
	       sum = sum + m_DynamicObjectsList[d_index].vertex[ m_DynamicObjectsList[d_index].faces[j].vertex_Order[f] ];
         }

         m_DynamicObjectsList[d_index].faces[j].centroid = (1.0f/m_DynamicObjectsList[d_index].faces[j].num_vertex)*sum;

         float3  V1 = m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[1]] -
        		 m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[0]];

         float3  V2 = m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[2]] -
        		 m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[1]];


         float3  V3 = m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[3]] -
        		 m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[2]];


         float3  V4 = m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[0]] -
        		 m_DynamicObjectsList[d_index].vertex[m_DynamicObjectsList[d_index].faces[j].vertex_Order[3]];


         float3 n         = cross(V2,V1);

         m_DynamicObjectsList[d_index].faces[j].normal = n/length(n);
         m_DynamicObjectsList[d_index].faces[j].area= 0.500000f*length(n) + 0.500000f*length(cross(V3,V4));

     }



     /* Rotate COM */
     float3 point = m_DynamicObjectsList[d_index].COM - about;
     m_DynamicObjectsList[d_index].COM.x = cos(angle)*point.x + sin(angle)*point.y;
     m_DynamicObjectsList[d_index].COM.y = -sin(angle)*point.x + cos(angle)*point.y;

     m_DynamicObjectsList[d_index].COM   = m_DynamicObjectsList[d_index].COM + about;




}



/*---------------------------------------------------------------------------*/

void KSimulationData::Repeat_Lifters()
{


	 /* Make Lifters */
		  int num_lifterTypes =m_num_DynamicObjects;

		  int num_liters = m_num_DObjectsPerType;


		  if(num_liters>0)
		  {
			  int off=0;
			  float theta = 4.0*asin(1.0f)/(float)num_liters;

			  /* Rotate second lifter Object */
			  if(num_lifterTypes==2)
			  {
				Make_New_Lifter( make_float3(m_WorldObjectsList[0].cyl_geo.radius,m_WorldObjectsList[0].cyl_geo.radius,0.0f),1,2,0.50f*theta);
				m_DynamicObjectsList[1] = m_DynamicObjectsList[2];
				off=1;

			  }


			  //printf("num objects %d num rep %d\n",m_num_KDynamicObjects,num_liters);
			  int dindex=1+off;
			 int ee=0;
			  /* now make addtional objects for each type*/
			  for(int i=0;i<num_lifterTypes;i++)
			  {
	            if(i==0)
	            {
	            	ee=1;
	            }

				for(int j=1;j<num_liters-1+ee;j++)
				{

	              Make_New_Lifter( make_float3(m_WorldObjectsList[0].cyl_geo.radius,m_WorldObjectsList[0].cyl_geo.radius,0.0f),i,dindex,theta*j);


	              if(num_lifterTypes==2)
	              {
	                dindex+=2;
	              }
	              else
	              {
	            	 dindex++;
	              }

				}

	             dindex=3;

			   }


			  m_num_DynamicObjects*=num_liters;

			  printf("num lifters %d \n",m_num_DynamicObjects);


		  }


}



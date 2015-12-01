#include "main.h"

#include <fstream>
#include "Graphics/Graphics.h"

#include <time.h>



/* 17/01/2015: Comments and clean up
 * 31/01/2015: adding run-time limit flag instead of batch mode */


/*---------------------------------------------------------------------------*/
/*                    1. Update Simulation on Device                         */
/*---------------------------------------------------------------------------*/
void UpdateSim()
{

	if(!Gl_bPause )
	{

	   /* Particle Wall Forces */
	   if(m_OpenGL.is_surface_tally)
	   {
		  m_WallForceSteps++;
	      /* Surface Forces */
	      if(m_WallForceSteps==m_OpenGL.surface_tally_calc_steps)
	      {
            Write_SurfaceForce();
            m_WallForceSteps=0;
	      }
	   }


		/* 1. Get energy of the system */
		if (m_OpenGL.is_energy_calc && m_Current_StepNumber>0 && (m_Current_StepNumber%m_OpenGL.energy_calc_steps)==0)
		{
			m_KDevice->IDevice_Get_Energy(energy);
			cout<<"time: "<<simtime_elapsed<<
		    " Energy:  Total = "<<(energy[0]+energy[1] + energy[2])
		    << "  Particle-Particle = "<< (energy[0])
		    << "  Particle-World = "<<energy[2]<<
		     "  Particle-DObjects = "<<energy[1]<<endl;
		}

		/* 2. Mill Simulation */
	    if( m_Sim_Type==ballmill )
	    {
	       sprintf(fps, "Rotation Time: %3.1f (s)",simtime_elapsed);

	       /* End of a revolution */
		   if(m_Current_StepNumber>0 && m_Current_StepNumber%m_Mill_SimData.Num_Steps_PerRev==0 )
		   {

			  /* 2.1  Call to the DEVICE to get system state */
			  m_KDevice->IDevice_Get_Energy(energy);

			  if(SimInfo.EnergyCalc)
			  {

			    cout<<"time: "<<simtime_elapsed<<
		        " Power:   Total = "<<( (energy[0]+energy[1] + energy[2])/
		        		(m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t))
		        << "  Particle: "<< (energy[0])/
		        (m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t)
		        << "  Lifter: "<<energy[1]/
		        (m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t)<<
		         " Drum "<<energy[2]/
		         (m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t)<<endl;


			    /* Write to file */
			    ResultFile<<"time: "<<simtime_elapsed<<
				        " Power:   Total = "<<( (energy[0]+energy[1] + energy[2])/
				        		(m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t))
				        << "  Particle: "<< (energy[0])/
				        (m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t)
				        << "  Lifter: "<<energy[1]/
				        (m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t)<<
				         " Drum "<<energy[2]/
				         (m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t)<<endl;

			  }

		 	   /* Output system information for post processing */
			   Output_SystemState(mill_EndRev,1,3);

		 	  if( m_OpenGL.is_runLimit && simtime_elapsed>=m_OpenGL.run_time)
		 	  {

		 		//t_rot-=SimInfo.InitalDelta_t;
		 	    printf("INFO: Auto Simulation part ended time %f\n",simtime_elapsed);
		 	    Output_SystemState(restart,1,3);

				 m_KDevice->clean();
				 delete [] m_d_P_ID;
				 delete [] m_d_Particle_Pos;
				 delete [] m_d_Particle_Quart;
				 delete [] m_d_Particle_Type;
				 delete [] m_colorP;
				 LogFile.close();

		 	   	exit(0);

		 	   }

		      m_KDevice->IDevice_Mill_reset_energy(0.0f);
			  Revoution_number++;

		     }/* End rev info */

		     /* Rotate Lifters about Z axis */
		     if(m_dynamic_rotation)
		     {
		    	Rotation_Angle += m_Mill_SimData.RadPerStep;
                //printf("%f  %f\n",Rotation_Angle,m_Mill_SimData.RadPerStep);

				Rotate_VolObject_Inc(make_float3(m_KWorldObject[0].cyl_geo.radius,
						               m_KWorldObject[0].cyl_geo.radius,0.0f));
				/* running time for mill revolution */
				num_WrotSeps++;


		     }

	    }/* End Mill simulation info */
	    else if(m_Sim_Type==silo)
	    {
	      sprintf(fps, "Elapsed Time: %3.2f (s)",simtime_elapsed);
	      //m_Current_StepNumber++;

	       if(m_Silo_SimData.measure_flow && m_Silo_SimData.is_hatch_open)
	 	   {


	 		   if(m_Silo_SimData.flow_count>=m_Silo_SimData.flow_steps)
	 		   {

	 			  m_Silo_SimData.flow_count=0;

	              ResultFile<<simtime_elapsed<<" "<<((Init_NumParticles - m_Silo_SimData.num_particles_rem)/(float)Init_NumParticles)*100.0<<endl;
	     	      m_Silo_SimData.flow_rate =  m_Silo_SimData.num_particles_rem;
	 		   }
	 		   m_Silo_SimData.flow_count++;


	 	   }

	    } /* If we rotating a world object */
	    else if( SimInfo.Rotating_WObject && m_dynamic_rotation )
		{
	    	Rotation_Angle+=m_Mill_SimData.RadPerStep;

			Rotate_WObject_inc( make_float3(m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.radius,0.0f) );

			if(m_num_KDynamicObjects > 0  )
			{
					Rotate_VolObject_Inc( make_float3(m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.radius,0.0f) );
			}

			if(m_OpenGL.render)
			{
			   Draw_WorldGeometry();
			}
			num_WrotSeps++;

			sprintf(fps, "Rotating Time: %3.1f (s)",simtime_elapsed);
		}
	    else /* Normal Simulation */
	    {
	    	sprintf(fps, "Elapsed Time: %3.1f (s)",simtime_elapsed);
	    }


	    /* User Interactions */
	    if(Translate_VObj)
	    {
	      Translate_VolObject(0,m_KDynamicObject[0].velocity*SimInfo.InitalDelta_t);
	    }



	    /* Check if we must write intermediate files */
	      if (m_OpenGL.write_intermFile>0)
	      {
	    	if(m_Current_StepNumber%m_OpenGL.write_intermFile==0)
	    	{
	    		//m_Current_StepNumber=0;
	    		printf("Info: %f Writing step data to file \n ",simtime_elapsed);
	    		Output_SystemState(step,0,3);
	    	}
	      }

		  /* Run to specified time if interactive mode */
		   if(m_OpenGL.total_SimTime>0.0f && !m_OpenGL.is_runLimit)
		   {
		 	  if( simtime_elapsed>=m_OpenGL.total_SimTime)
		 	  {
		 	     printf("INFO: Interactive Simulation Reached time \n");
			 	 Output_SystemState(restart,0,3);
				 m_KDevice->clean();
				 delete [] m_d_P_ID;
				 delete [] m_d_Particle_Pos;
				 delete [] m_d_Particle_Quart;
				 delete [] m_d_Particle_Type;
				 delete [] m_colorP;
				 LogFile.close();
		 	   	exit(0);

		 	   }
		   }/* Non-interactive */
		   else if (m_OpenGL.is_runLimit && m_OpenGL.run_time>0.0f)
		   {
			 	  if( simtime_elapsed>=m_OpenGL.run_time)
			 	  {

			 		//simtime_elapsed-=SimInfo.InitalDelta_t;
			 	    printf("INFO: Auto Simulation part ended time %f\n",simtime_elapsed);
			 	    Output_SystemState(restart,0,3);

					 m_KDevice->clean();
					 delete [] m_d_P_ID;
					 delete [] m_d_Particle_Pos;
					 delete [] m_d_Particle_Quart;
					 delete [] m_d_Particle_Type;
					 delete [] m_colorP;
					 LogFile.close();

			 	   	exit(0);

			 	   }
		   }

		   /* running time over total */
		   simtime_elapsed +=  SimInfo.InitalDelta_t;
		   m_Current_StepNumber++;



		  /* GPU: Calculates the forces and net velocity on all particles */
		  m_KDevice->IDevice_DEM_UpdateSim();



	 }/* If not paused */


}
/*---------------------------------------------------------------------------*/


void Silo_Stats()
{
	  if( m_Silo_SimData.flow_count>=m_Silo_SimData.flow_steps)
	  {

		  m_Silo_SimData.num_particles_rem=0;
	    /* 2.1  Call to the DEVICE to get system state */
	    m_KDevice->IDevice_Get_Positions( m_d_Particle_Pos, m_d_Particle_Quart,
	    		                          m_d_Particle_Type,
	    		                          m_d_P_ID );

	   /* Loop through all particles and update there position on screen */

	   for ( int i=0; i<SimInfo.Num_Particles; i++ )
	   {


			   /* Count number left in Silo */
			   if(m_d_Particle_Pos[i].y>m_Silo_SimData.hatch_height)
			   {
				 m_Silo_SimData.num_particles_rem++;
			   }

	    } /* End loop over all particles */

	  }/* frame check */

}


/*---------------------------------------------------------------------------*/
/*         2. Entry point of the CODE everything is called from here         */
/*---------------------------------------------------------------------------*/
void display()
{
   /* 0. Set Camera and Viewport */
   GL_DisplaySet();


   if(!m_OpenGL.ortho)
   {
     /* 1. Draw World Geometry */
     glPushMatrix();
	   glScalef( GLScale.x, GLScale.y, GLScale.z );
	   glCallList(1);
	   displayAxis();
     glPopMatrix();

     /* 2. Draw Dynamic Geometry */
     for ( int i=0; i<m_num_KDynamicObjects; i++ )
     {

       glPushMatrix();
  	     glScalef( GLScale.x, GLScale.y, GLScale.z );
  	     glCallList(50+i);
       glPopMatrix();
     }
   }
   else /* Ortho */
   {
	   glPushMatrix();
	   glTranslatef(-m_KWorldObject[0].cyl_geo.radius,-m_KWorldObject[0].cyl_geo.radius,0.0);

	  /* 1. Draw World Geometry */
		glCallList(1);
		displayAxis();
	  glPopMatrix();

	  /* 2. Draw Dynamic Geometry */
	   for ( int i=0; i<m_num_KDynamicObjects; i++ )
	   {
	     glPushMatrix();
	     glTranslatef(-m_KWorldObject[0].cyl_geo.radius,-m_KWorldObject[0].cyl_geo.radius,0.0);
	  	  glCallList(50+i);
	     glPopMatrix();
	   }
   }

  frame++;
  if( frame==FPS )
  {

	m_Silo_SimData.num_particles_rem = 0;

	/* 2.1  Call to the DEVICE to get system state */
    m_KDevice->IDevice_Get_Positions( m_d_Particle_Pos,m_d_Particle_Quart,
    		                          m_d_Particle_Type, m_d_P_ID          );

    if(m_d_P_ID[0]!=-2)
    {

    if(m_OpenGL.color_type==1)/* Color by velocity */
    {
      /* Get velocities for color */
      m_KDevice->IDevice_Get_Particle_Velocity( m_d_Particle_Vel,m_d_Particle_RVel,0,m_d_Particle_Type, m_d_P_ID );
    }

   /* Loop through all particles and update there position on screen */

   for ( int i=0; i<SimInfo.Num_Particles; i++ )
   {

	   if(m_Silo_SimData.measure_flow && m_Silo_SimData.is_hatch_open)
	   {
		   /* Count number left in Silo */
		   if(m_d_Particle_Pos[i].y>m_Silo_SimData.hatch_height)
		   {
			 m_Silo_SimData.num_particles_rem++;
		   }
	   }
       if (m_d_P_ID[i]!=-1 )//(m_d_P_ID[i]==13 || m_d_P_ID[i]==4)
       {
		 glPushMatrix();
         if(m_OpenGL.ortho)
         {
        	if(m_Sim_Type==ballmill)
        	{
              glTranslatef(-m_KWorldObject[0].cyl_geo.radius,-m_KWorldObject[0].cyl_geo.radius,0.0);
        	}
         }
         else
         {
            glScalef(GLScale.x, GLScale.y,GLScale.z);
         }


           glTranslatef( m_d_Particle_Pos[i].x,m_d_Particle_Pos[i].y,
        		         m_d_Particle_Pos[i].z                          );


           /* Drawing done with COM at (0,0,0) */

           /* Rotate if Polyhedra  */
           if(Gl_WireframeP)
           {
               /* Convert to Axis Angle for OpenGL */
      	       axisangle = quart_axisAngleH(m_d_Particle_Quart[i]);
           }

           /* Rotate if Polyhedra  */
           if(m_sim_particle_type==1 )
           {
               /* Convert to Axis Angle for OpenGL */
      	       axisangle = quart_axisAngleH(m_d_Particle_Quart[i]);

                /* Drawing done with COM at (0,0,0) */
				 glTranslatef( m_KParticleObject[ m_d_Particle_Type[i] ].COM.x,
							 m_KParticleObject[ m_d_Particle_Type[i] ].COM.y,
							 m_KParticleObject[ m_d_Particle_Type[i] ].COM.z);

				 glRotatef( axisangle.w,axisangle.x,axisangle.y,axisangle.z );

				 glTranslatef( -m_KParticleObject[ m_d_Particle_Type[i] ].COM.x,
							 -m_KParticleObject[ m_d_Particle_Type[i] ].COM.y,
							 -m_KParticleObject[ m_d_Particle_Type[i] ].COM.z);

           }

           if(m_OpenGL.color_type==1)/* Color by velocity */
           {
				int colindex = floor(length(m_d_Particle_Vel[i]/100.0f)/m_OpenGL.color_bin);

				if(colindex>5)
				{
				  colindex=5;
				}

				if(colindex<0)
				{
				  colindex=0;
				}

				 /* Draw particle */
				 glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, VelCol[colindex]);
				 glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
				 glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
				 glMaterialf(GL_FRONT, GL_SHININESS, shininess);
           }

           glCallList(10+m_d_Particle_Type[i]);


         glPopMatrix();
      }

    } /* End loop over all particles */
    }
    glFlush();

    if(m_OpenGL.color_type==1)
    {
      drawColorBar();
    }

    glutSwapBuffers();


    frame = 0;
  }/* frame check */


     /* Print actual time to window */
     glutSetWindowTitle(fps);

     /* Take another step */
     UpdateSim();

     glutPostRedisplay();

}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
void Set_SimInfo()
{

   SimInfo.Num_Particles   = m_KSimulationData->get_num_Particles();

   m_colorP                = new float3    [SimInfo.Num_Particles]; /* HOST */

   m_d_Particle_Pos        = new float3    [SimInfo.Num_Particles]; /* <--DEVICE */
   m_d_Particle_Quart      = new Quaterion [SimInfo.Num_Particles]; /* <--DEVICE */
   m_d_P_ID                = new int       [SimInfo.Num_Particles]; /* <--DEVICE */
   m_d_Particle_Type       = new uint      [SimInfo.Num_Particles]; /* <--DEVICE */
   m_d_Particle_Vel        = new float3    [SimInfo.Num_Particles]; /* <--DEVICE */
   m_d_Particle_RVel       = new float3    [SimInfo.Num_Particles]; /* <--DEVICE */
   m_d_Particle_Acc        = new float3    [SimInfo.Num_Particles]; /* <--DEVICE */

   m_dc_WallForces         = new float3    [SimInfo.Num_Particles]; /* <--DEVICE */
   m_dc_WallContacts       = new float3    [SimInfo.Num_Particles]; /* <--DEVICE */


   cout<<" "<<std::endl;
   cout<<"INFO: Copying Data into Structures "<<std::endl;
   cout<<" "<<std::endl;

                /* Set simulation Info*/
   SimInfo.Num_WorldObjects       = m_KSimulationData->get_num_WorldObjects();

   SimInfo.Num_ParticleObjects    = m_KSimulationData->get_num_ParticleObjects();
   SimInfo.Num_ParticlesPerObject = m_KSimulationData->get_num_ParticlesPerObject();
   SimInfo.particle_type          = m_sim_particle_type;

   SimInfo.InitalDelta_t          = m_KSimulationData->get_delta_t();
   SimInfo.Force_Field            = m_KSimulationData->get_gravity();
   SimInfo.Rotation               = m_KSimulationData->get_isRotation();

   SimInfo.Roll_Res               = m_KSimulationData->get_RollRes();
   SimInfo.Global_VisDamp         = m_KSimulationData->get_GlobalDamp();

   SimInfo.Rotating_WObject       = m_KWorldObject[0].is_rotating;


   if (SimInfo.Rotating_WObject)
   {
	   printf("rotating world\n");
   }


                     /* INITAL POS */
   m_InitPos.use_device      = cuda_device;
   m_InitPos.multi_gpu       = use_multi_gpu;
   m_InitPos.num             = m_KSimulationData->get_InitPosGrid().num;
   m_InitPos.start           = m_KSimulationData->get_InitPosGrid().start;
   m_InitPos.space           = m_KSimulationData->get_InitPosGrid().space;
   m_InitPos.grid_type       = m_KSimulationData->get_InitPosGrid().grid_type;
   m_InitPos.threads_perBlock = 1024; // KEPLER ARCHITECTURE
   //m_InitPos.threads_perBlock = 196; // MAXWELL ARCHITECTURE

   if(m_sim_particle_type==1)
   {
     /* Set particle sizes 0 has the largest */
     for( int i=0; i<m_KSimulationData->get_num_ParticleObjects();i++)
     {
	   m_InitPos.p_size[i] = m_KSimulationData->get_InitPosGrid().p_size[i];

      }
   }
   else
   {
	  for( int i=0; i<m_KSimulationData->get_num_ParticleObjects();i++)
	  {
		 m_InitPos.p_size[i] = make_float3( m_KParticleObject[i].radius*2.0f,
				                            m_KParticleObject[i].radius*2.0f,
				                            m_KParticleObject[i].radius*2.0f );
	  }
   }

                         /* NN GRID */
     SimInfo.cellSize          = m_KSimulationData->get_NNGrid().size;
     SimInfo.worldOrigin       = m_KSimulationData->get_NNGrid().origin;
     float3 worldsize          = m_KSimulationData->get_worldSize();

     /* Sane check */
     float P_bound = m_KParticleObject[0].radius*2.00f;

     if( (m_KSimulationData->get_NNGrid().size.x< P_bound) ||
         (m_KSimulationData->get_NNGrid().size.y< P_bound)||
         (m_KSimulationData->get_NNGrid().size.z< P_bound)  )
     {
    	 cout<<"!INFO-ERROR: NN Dimensions: to small max particle =  "<<P_bound<<std::endl;
    	 //exit(1);
     }

     LogFile<<"World Size: "<<worldsize.x<<","<<worldsize.y<<","<<worldsize.z<<std::endl;

     /* Calculate Num Cells */
     float3 TotalSize = worldsize - m_KSimulationData->get_NNGrid().origin;

     LogFile<<"Grid Dimensions: "<<TotalSize.x<<","<<TotalSize.y<<","<<TotalSize.z<<std::endl;

     SimInfo.num_NNCells.x = (int)ceil(TotalSize.x/m_KSimulationData->get_NNGrid().size.x);
     SimInfo.num_NNCells.y = (int)ceil(TotalSize.y/m_KSimulationData->get_NNGrid().size.y);
     SimInfo.num_NNCells.z = (int)ceil(TotalSize.z/m_KSimulationData->get_NNGrid().size.z);

     LogFile<<"Grid Cells NUM: "<<SimInfo.num_NNCells.x<<","<<SimInfo.num_NNCells.y<<","<<SimInfo.num_NNCells.z<<std::endl;
     LogFile<<" "<<std::endl;


     m_InitPos.use_file = m_KSimulationData->get_readFile();

     SimInfo.unit_test = m_KSimulationData->get_isDebug();

     if(m_KSimulationData->get_isDebug()>0)
     {
       cout<<"Unit Test Mode" <<std::endl;
     }


     LogFile<<"Rotation: "<<SimInfo.Rotation<<std::endl;



      m_InitPos.velocity[0]           = m_KSimulationData->get_InitPosGrid().velocity[0];
      m_InitPos.launch_Vel = m_KSimulationData->get_InitPosGrid().launch_Vel;
      m_InitPos.fill_plane_start = m_KSimulationData->get_InitPosGrid().fill_plane_start;
      m_InitPos.fill_plane_end = m_KSimulationData->get_InitPosGrid().fill_plane_end;

       /* Now store particle types on HOST */

	   int index_particle = 0;

	   /* store inverse inertia tensor for sphere */

	   LogFile<<"INFO: Spring Paramaters "<<std::endl;


	   double Total_PMass   = 0.0;
	   double Total_PVolume = 0.0;

	   for(int i=0; i<SimInfo.Num_ParticleObjects; i++)
	   {
		   Total_PMass   += m_KParticleObject[i].mass*m_KSimulationData->get_num_ParticlesPerObject()[i];
		   Total_PVolume += PI*((4.0f/3.0f)*powf(m_KParticleObject[i].radius,3))*m_KSimulationData->get_num_ParticlesPerObject()[i];

		   LogFile<<"Particle: "<<i<<std::endl;

           if(SimInfo.particle_type==0)
           {
		   m_KParticleObject[i].InertiaT[0] = 1.0f/(0.40f*m_KParticleObject[i].radius*
				                m_KParticleObject[i].radius*m_KParticleObject[i].mass);
           }
           else
           {


        	   for(int g=0;g<9;g++)
        	   {
        		   m_KParticleObject[i].InertiaT[g] = (1.0f/(0.40f*m_KParticleObject[i].radius*
        				                m_KParticleObject[i].radius*m_KParticleObject[i].mass))*m_KParticleObject[i].InertiaT[g];
        	   }
           }

		   LogFile<<" SURFACE "<<std::endl;

		   LogFile<<" Kn:    "<<m_KParticleObject[i].surface_Kn <<std::endl;
		   LogFile<<" Cn:    "<<m_KParticleObject[i].surface_Cn <<std::endl;

		   LogFile<<" Fric:  "<<m_KParticleObject[i].surface_Fric_static<<std::endl;

		   LogFile<<" "<<std::endl;

		   LogFile<<" Lifter "<<std::endl;
		   LogFile<<" Kn:    "<<m_KParticleObject[i].Dsurface_Kn <<std::endl;
		   LogFile<<" Cn:    "<<m_KParticleObject[i].Dsurface_Cn <<std::endl;

		   LogFile<<" Fric:  "<<m_KParticleObject[i].Dsurface_Fric_static<<std::endl;

		   LogFile<<" "<<std::endl;

		   for(int j=0; j<SimInfo.Num_ParticleObjects; j++)
		   {
			   LogFile<<"PP: "<<j<<std::endl;
			   LogFile<<" Kn:    "<<m_KParticleObject[i].PP[j].Kn <<std::endl;
			   LogFile<<" Cn:    "<<m_KParticleObject[i].PP[j].Cn <<std::endl;
			   LogFile<<" Fric:  "<<m_KParticleObject[i].PP[j].Fric_static<<std::endl;

		   }

		   LogFile<<" "<<std::endl;
		   LogFile<<" "<<std::endl;

	   }


	   LogFile<<"INFO: Data Structures populated "<<std::endl;
	   LogFile<<" "<<std::endl;

	     /* Get Simulation Specfic Data */
	   m_Sim_Type = m_KSimulationData->get_simType();

       SimInfo.Vel_Limit=m_KSimulationData->get_velLimit();

       if ( SimInfo.Vel_Limit>0.0f)
       {
       	 printf("!Warning Limiting Max Collision velocity to : %f\n",SimInfo.Vel_Limit);
       }

	     if( m_Sim_Type==ballmill )
	     {

	    	m_Mill_SimData.Current_StepNumber = 0;

	        SimInfo.Simulation_Type = ballmill;

	        SimInfo.isMillSim = true;
	        m_Mill_SimData = m_KSimulationData->get_millObject();

	        if( m_Mill_SimData.power_output_revs > 0 )
	        {
	          cout<<"Power Calc: True "<<std::endl;
	          SimInfo.EnergyCalc = true;
	        }
	        else
	        {
	          cout<<"Power Calc: False "<<std::endl;
	      	  SimInfo.EnergyCalc = false;
	        }

		   /* Mill Rotation Speed (Rad/s) */

		  	 m_Mill_SimData.Num_Steps_PerRev = (60.0f/m_Mill_SimData.mill_RPM)/SimInfo.InitalDelta_t;

		  	 if(m_OpenGL.total_SimTime<=0.0)
		  	 {

		  	  m_OpenGL.total_SimTime = m_Mill_SimData.total_revs*m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t;
		  	 }
		     LogFile<<"Total SimTime: "<<m_OpenGL.total_SimTime<<endl;


		    LogFile<<"num Power steps: "<<m_Mill_SimData.Num_Steps_PerRev<< " time: "<<m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t<<endl;


		      /* We want RAD/S: so convert from rpm to rad/s to rad/step*/

		      m_Mill_SimData.RadPerStep =  -( (m_Mill_SimData.mill_RPM*2.000f*PI)/60.000f)*SimInfo.InitalDelta_t;

		      LogFile<<"INFO: Mill Speed (Rad/Step): "<<m_Mill_SimData.RadPerStep << " (Rad/S): " << m_Mill_SimData.RadPerStep /SimInfo.InitalDelta_t<<std::endl;
		      LogFile<<" "<<std::endl;

	    	  Vel_Mill = fabs((m_Mill_SimData.RadPerStep /SimInfo.InitalDelta_t)*m_KWorldObject[0].cyl_geo.radius*0.01);
			  printf("Vel Mill %f\n",Vel_Mill);


				SimInfo.MillRotVel = -(Vel_Mill);

         if(m_num_KDynamicObjects>0)
         {
           SimInfo.DObject_CylinderBoundR = m_KWorldObject[0].cyl_geo.radius - (2.0f*m_KDynamicObject[0].COM.y);
           SimInfo.cent = make_float3(m_KWorldObject[0].cyl_geo.center_bot_cap.x,m_KWorldObject[0].cyl_geo.center_bot_cap.y,0.0f);
         }

	   }
	   else if(m_Sim_Type==silo)
	   {
	        SimInfo.isMillSim = false;
	        m_Silo_SimData = m_KSimulationData->get_siloObject();

	        if (m_Silo_SimData.measure_flow)
	        {
	          printf("INFO: Flow Counter position at %f at %f \n",m_KWorldObject[m_num_KWorldObjects-1].vertex[0].y,m_Silo_SimData.flow_steps*SimInfo.InitalDelta_t);
	        }

	        m_Silo_SimData.hatch_height = m_KWorldObject[m_num_KWorldObjects-1].vertex[0].y;
	        m_Silo_SimData.flow_count=0;

	        SimInfo.Silo_Kill_Height = m_Silo_SimData.hatch_height-5.0f;
	        SimInfo.Kill_Particles=m_Silo_SimData.kill_particle;

	        if(SimInfo.Kill_Particles)
	        {
	        	printf("INFO: Kill Particles \n");
	        }

	        if(m_Silo_SimData.manual_hatch)
	        {
	          m_Silo_SimData.is_hatch_open=true;
	          printf("INFO: Hatch Open \n");
	        }
	        else
	        {
	          m_Silo_SimData.is_hatch_open=false;

	          printf("INFO: Hatch Closed \n");
	        }

	         m_Silo_SimData.num_particles_rem = SimInfo.Num_Particles;

	        if(m_OpenGL.total_SimTime==0.0)
	        {
	        	m_OpenGL.total_SimTime=60.0f;

	        	if(m_Silo_SimData.measure_flow)
	        	{
		          printf("INFO: Run Till Silo Empty\n");
	        	}
	        	else
	        	{
	        		printf("ERROR: Please specify counter or time \n");
	        		exit(1);
	        	}
	        }

	   }
	   else if( SimInfo.Rotating_WObject )
	    {
           m_Mill_SimData = m_KSimulationData->get_millObject();
		   m_Mill_SimData.Num_Steps_PerRev = (60.0f/m_Mill_SimData.mill_RPM)/SimInfo.InitalDelta_t;

		   if(m_OpenGL.total_SimTime<=0.0)
		   {
			  m_OpenGL.total_SimTime = m_Mill_SimData.total_revs*m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t;
		   }
		   cout<<"Total SimTime: "<<m_OpenGL.total_SimTime<<endl;

	       /* We want RAD/S: so convert from rpm to rad/s to rad/step*/
		   m_Mill_SimData.RadPerStep =  -( (m_Mill_SimData.mill_RPM*2.000f*PI)/60.000f)*SimInfo.InitalDelta_t;

		   LogFile<<"INFO: Mill Speed (Rad/Step): "<<m_Mill_SimData.RadPerStep << " (Rad/S): " << m_Mill_SimData.RadPerStep /SimInfo.InitalDelta_t<<std::endl;
		   LogFile<<" "<<std::endl;


		   Vel_Mill = fabs((m_Mill_SimData.RadPerStep /SimInfo.InitalDelta_t)*(m_KWorldObject[0].cyl_geo.radius-100.0f)*0.01);
		   printf("Vel Mill (m/s) %f\n",Vel_Mill);

		   SimInfo.MillRotVel = -(Vel_Mill);

	     }




	   /* General options used for development */
	   SimInfo.Integration_Type = 0;
	   SimInfo.use_symmetry     = false; /* For multi GPU symmetry will be problematic */
	   SimInfo.sphere_orient    = Gl_WireframeP;

	   if(SimInfo.Rotating_WObject)
	   {
	     SimInfo.Kill_Particles = false;
	   }


       SimInfo.max_size = make_float3( SimInfo.num_NNCells.x*SimInfo.cellSize.x,
       		                        SimInfo.num_NNCells.y*SimInfo.cellSize.y,
       		                        SimInfo.num_NNCells.z*SimInfo.cellSize.z);

       printf("MAX SIZE %f %f %f \n",SimInfo.max_size.x,
       		                      SimInfo.max_size.y,
       		                      SimInfo.max_size.z);

       printf("Total Mass(kg): %f  Total Volume(cm3): %f \n",Total_PMass,Total_PVolume);

       SimInfo.use_hist = false;

}


int main(int argc, char **argv)
{

	 fstream SimNameF;
	 string dumS;
	 int mode2D,num_devices;


	 /* Get the world name */
	 SimNameF.open("../Simulation.KSim");

	 /* Read run information */
	 SimNameF>>dumS>>ProjectFolder>>dumS>>WorldName>>dumS
	        >>mode2D>>dumS>>dumS>>cuda_device>>dumS>>num_devices;

	 if( num_devices > 0 )
	 {
		use_multi_gpu = true;
	 }


	 SimNameF.close();

     string Folder;
	 system("mkdir  ../Results");

     /* Make Folder for output */
     Folder =  ( "mkdir  ../Results/" + ProjectFolder ).c_str();
	 system(Folder.c_str());

     Folder =  ( "mkdir  ../Results/" + ProjectFolder +"/"+ WorldName ).c_str();
	 system(Folder.c_str());


	 /* Open Logfile */
	 LogFile.open( ("../Results/"+ProjectFolder +"/"+ WorldName +"/Simulation.Log").c_str(),LogFile.app);



	 /* Read In World File */
     m_KSimulationData = new KSimulationData(ProjectFolder,WorldName);



     OpenGLOBJ GLOptions= m_KSimulationData->get_OpenGLObject();
     FPS=GLOptions.FPS;

	 /* Set which GPU to use: There is Only 1 on the lap-top DEVICE=GPU */
	 if(num_devices>0)
	 {
		 printf("INFO: Using %d GPUS\n",num_devices );
	 }
	 else
	 {
		 printf("INFO: Using GPU %d \n",cuda_device );
	 }

     if(mode2D==1)
	 {
    	 SimInfo.Mode2D = true;
		 printf("INFO: Code is running in 2D mode!!\n");
	 }
     else
     {
    	 SimInfo.Mode2D = false;
    	 printf("INFO: Code is running in 3D mode!!\n");
     }

	 m_num_KWorldObjects    = m_KSimulationData->get_num_WorldObjects();
	 m_num_KParticleObjects = m_KSimulationData->get_num_ParticleObjects();
	 m_num_KDynamicObjects  = m_KSimulationData->get_num_DYParticleObjects();
	 printf("numdobjects %d \n",m_num_KDynamicObjects);


     /* Store Data from Input Class */
	 m_OpenGL               = m_KSimulationData->get_OpenGLObject();
	 m_sim_particle_type    = m_KSimulationData->get_particleType();
	 read_initialPos_file   = m_KSimulationData->get_readFile();

	 m_KParticleObject      = m_KSimulationData->get_particleObject();

	 m_KWorldObject         = m_KSimulationData->get_worldObject();

	 m_KDynamicObject        = m_KSimulationData->get_dynamicObject();



	 Init_NumParticles = m_KSimulationData->get_num_Particles();

     //if(m_KWorldObject[0].is_rotating)
     //{
       m_KSimulationData2 = new KSimulationData(ProjectFolder,WorldName);
	   m_KWorldObject_Initial = m_KSimulationData2->get_worldObject();
	   m_KDynamicObject_Initial = m_KSimulationData2->get_dynamicObject();
     //}


	 /* Set information on the GPU */
	 LogFile<<"INFO: Setting SimINFO "<<endl;
	 Set_SimInfo();


	 LogFile<<"INFO: Invoking CUDA Device "<<endl;



	 /* Create a new instance to the Device with geometric information */
	 m_KDevice = new KDevice( m_KWorldObject, m_KParticleObject,
			                  m_KDynamicObject,m_num_KWorldObjects,
			                  m_num_KParticleObjects,
							  m_num_KDynamicObjects,
							  &SimInfo,&m_InitPos                    );




	 /* Setup the initial state */
	 if(!m_OpenGL.is_runLimit)
	 {
		 printf("INFO: Starting User Defined simulation \n");

		 /* Read Initial Data from File */
		 if( read_initialPos_file==1 )
		 {
		   Read_FromFile( m_KSimulationData->get_filename() );
		 }

		 if( m_Sim_Type==ballmill )
		 {
	     	 m_KDevice->IDevice_Mill_reset_energy(0.0f);
		 }

	 }/* Automatated restart Run for a specified time then wait for GPU to cool */
	 else
	 {

		 printf("INFO: Starting Automated simulation \n");

		 Gl_bPause          = false;


        /* Simulation Type settings */
		 if(m_Sim_Type==ballmill)
		 {
		   m_dynamic_rotation = true;
		   m_KDevice->IDevice_drum_rotation(true);
		 }
		 else
		 {
		   m_dynamic_rotation = false;
		 }

		 if(SimInfo.Simulation_Type==silo)
		 {
		   m_Silo_SimData.is_hatch_open=true;
		   m_KDevice->IDevice_OpenHatch(m_KWorldObject,m_num_KWorldObjects-1);
		 }

		 /* Read Initial Data from File if this is the first instance */
		 if( read_initialPos_file==1 && simtime_elapsed==0.0f )
		 {
		   printf("INFO: Starting non-graphic Auto Simulation \n");
		   Read_FromFile( m_KSimulationData->get_filename() );

		 }
		 else if( (simtime_elapsed<m_OpenGL.total_SimTime) )
		 {
			 /* Restart Read Data from state.tmp */
			 printf("INFO: Graphic Simulation Reading Previous State Auto Simulation \n");
			 Read_FromFile("state.tmp");
		 }

         /* Exit condition */
		 if((simtime_elapsed>=m_OpenGL.total_SimTime*0.9999f))
		 {
		   printf("Auto Simulation Ended Time \n");
		   exit(0);
		 }
	  }


	 Gl_bPause          = false;


     if (SimInfo.isMillSim)
     {
       if(m_InitPos.grid_type!=2)
       {
         printf("Reset Energy: \n" );
         m_KDevice->IDevice_Mill_reset_energy(0.0f);
         m_KDevice->IDevice_drum_rotation(true);
         m_dynamic_rotation = true;
       }
       else
       {
    	  m_dynamic_rotation = false;
       }
     }

	 /* Open results file */
	 if(m_Sim_Type==silo)
	 {
	   ResultFile.open( ("../Results/"+ProjectFolder +"/"+ WorldName + "/FlowRate.dat").c_str(),LogFile.app);
	 }
	 else
	 {
	   ResultFile.open( ("../Results/"+ProjectFolder +"/"+  WorldName + "/Results.dat").c_str(),LogFile.app);
	 }

     /* non graphics */
	 if( !m_OpenGL.render )
	 {

	    printf("INFO: Command line only No graphics  \n");
		Gl_bPause            = false;

		if(SimInfo.Simulation_Type==silo)
	    {
		  m_Silo_SimData.is_hatch_open=true;
		  m_KDevice->IDevice_OpenHatch(m_KWorldObject,m_num_KWorldObjects-1);
	    }

		 if( m_Sim_Type==ballmill )
		 {
			 m_dynamic_rotation = true;
			 m_KDevice->IDevice_drum_rotation(true);
		 }

         printf("Reset Energy: \n" );
         m_KDevice->IDevice_Mill_reset_energy(0.0f);

        printf("INFO: Total Simulation Time: %f\n",m_OpenGL.total_SimTime);

        while( simtime_elapsed<=m_OpenGL.total_SimTime )
        {
        	 /* Compute statstics */
      	     if(m_Sim_Type==silo && m_Silo_SimData.measure_flow)
      	     {
      	    	Silo_Stats();
      	     }
      	     /* Take another step for flow only */
      	     UpdateSim();
      	     printf("  %f \r",simtime_elapsed);
       }
	 }
	 else
	 {
		 /* OpenGL display */
		 if(m_OpenGL.render)
		 {

			     printf("INFO: Using openGL for display  \n");
				 /* Set Up OpenGL window */
				 glutInit(&argc, argv);
				 glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
				 glutInitWindowSize(width, height);
				 MainWindow = glutCreateWindow(" BLAZE-DEM V_07_2015 ");
				 glEnable(GL_DEPTH_TEST);
				 glClearColor(0.60, 0.40, 0.20, 0.0);
				 glutReportErrors();


				 /* 3D Perspective or Orthographic Projection (2D)*/
				 if(m_OpenGL.ortho)
				 {
					 GLScale = make_float3(1.0f,1.0f,1.0f);
				 }
				 else
				 {
					 GLScale = make_float3(0.050f,0.050f,0.050f);
				 }

			     glutDisplayFunc(display);

			     if(m_OpenGL.ortho)
			     {
				   glutReshapeFunc(ReshapeGL);
			     }
			     else
			     {
			    	glutReshapeFunc(ReshapeGLP);
			     }
				 glutMouseFunc(mouse);
				 glutMotionFunc(motion);
				 glutKeyboardFunc(keyboard);

				 /* Create Display Lists */
				 Draw_WorldGeometry();

				 if(m_sim_particle_type==spheres)
				 {
				   Draw_ParticleGeometry_Sphere();
				 }
				 else if(m_sim_particle_type==polyhedra)
				 {
				   Draw_ParticleGeometry_Poly();
				 }

				 if( m_num_KDynamicObjects > 0 )
				 {
					  Draw_DynamicGeometry();
				 }

				 /* OpenGL */
				 CreateEnvironment();
		 }

	     Gl_bPause            = true;


	   cout<<"Usage: 'space' is to start/stop simulation|  'p' is to dump system state | 'q' exits "<<endl;
	   cout<<"       's' is to start/stop mill rotation   and 'o' is to open/close the silo hatch "<<endl;

	   cout<<"       'w'/'e' is to zoom in/out or use mouse: : 1] Left + Right = Translate, 2] Right = Rotate,\n"
			     "Left + Right + Middle = Zoom "<<endl;



       glutMainLoop();
	 }

        printf("INFO: Simulation  Complete \n");
        printf("INFO: Cleaning Resources \n");
      	m_KDevice->clean();
	    ResultFile.close();

  return 0;

}




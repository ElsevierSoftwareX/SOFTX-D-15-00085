/*
 * FileIO.h
 *
 *  Created on: Jun 9, 2015
 *      Author: nicolin
 */

#ifndef FILEIO_H_
#define FILEIO_H_

/* Otype: 1 is for time step, 2 is mill rev */
void Output_SystemState(Output_Type Otype, int revflag, int Flag_OutputDetail)
{
	ofstream outF;

	stringstream tstep;
	tstep<< simtime_elapsed;

	/* Create output-file in specified location */

	if(Otype!=user && Otype!=mill_EndRev)
	{
	    Flag_OutputDetail = m_OpenGL.output_format;
	}


	/* If if was a filling then create a start File */
    if(m_KSimulationData->get_InitPosGrid().grid_type==2)
    {
    	outF.open( ("../Projects/"+ProjectFolder +"/"+ "Start"+WorldName ).c_str() );
    }
    else
	if(Otype==step)
	{
		outF.open( ("../Results/"+ProjectFolder +"/"+ WorldName + "/time" + tstep.str() ).c_str() );
	}
	else if(Otype==mill_EndRev)
	{
	  stringstream tstep;
	  tstep<< (Revoution_number-1);
	  outF.open( ("../Results/"+ProjectFolder +"/"+ WorldName  +"/Rev_" + tstep.str() ).c_str() );

	}
	else if(Otype==restart)
	{
	  outF.open( ("../Results/"+ProjectFolder +"/"+ WorldName +"/state.tmp" ).c_str() );
	}
	else
	{
	  outF.open( ("../Results/"+ProjectFolder +"/"+ WorldName +"/SystemState"+tstep.str()+".dat" ).c_str() );
	}


	int silo_count=0;
    float sumV=0.0f;
    float maxVel=0.0f;
    int   maxID=-1;





	m_KDevice->IDevice_Get_System_State( m_d_Particle_Pos, m_d_Particle_Quart, m_d_Particle_Vel,m_d_Particle_RVel,
			                             m_d_Particle_Acc,
			                             m_d_Particle_Type,m_d_P_ID);

    /* Create Header information for the File */

	  if( Flag_OutputDetail==1 )/* Position Only */
	  {
		 if(SimInfo.particle_type==0)
		 {
		   outF<<"#  P_ID    PType_ID   Pos(x,y,z)cm  "<<endl;
		 }
		 else
		 {
		   outF<<"#  P_ID    PType_ID   Pos(x,y,z)cm  Orient(w,x,y,z)"<<endl;
		 }
	  }
	  else
	  if(Flag_OutputDetail==2)/* Position and Velocity */
	  {
		 if(SimInfo.particle_type==0)
		 {
		   outF<<"#  P_ID   PType_ID   Pos(x,y,z)cm    Vel(x,y,z)cm/s    AngVel(x,y,z)cm/s"<<endl;
		 }
		 else
		 {
		   outF<<"#  P_ID   PType_ID   Pos(x,y,z)cm    Vel(x,y,z)cm/s  Orient(w,x,y,z) AngVel(x,y,z)cm/s"<<endl;
		 }
	  }
	  else
	  if(Flag_OutputDetail==3)
	  {
		 if(SimInfo.particle_type==0)
		 {
		   outF<<"#  P_ID   PType_ID   Pos(x,y,z)cm  Vel(x,y,z)cm/s     AngVel(x,y,z)cm/s"<<endl;
		 }
		 else
		 {
		  outF<<"#  P_ID   PType_ID   Pos(x,y,z)cm  Vel(x,y,z)cm/s    Orient(w,x,y,z)  AngVel(x,y,z)cm/s"<<endl;
		 }
	  }
	  else
	  if(Flag_OutputDetail==4)
	  {
			 if(SimInfo.particle_type==0)
			 {
			   outF<<"#  P_ID   PType_ID   Pos(x,y,z)cm  Vel(x,y,z)cm/s    AngVel(x,y,z)cm/s   | MagVel(m/s)  MagAVel(m/s) "<<endl;
			 }
			 else
			 {
			  outF<<"#  P_ID   PType_ID   Pos(x,y,z)cm  Vel(x,y,z)cm/s    Orient(w,x,y,z)  AngVel(x,y,z)cm/s  | MagVel(m/s)  MagAVel(m/s)"<<endl;
			 }
	  }
	  else
	  if(Flag_OutputDetail==5)
	  {
		  outF<<"# COM   Vertex"<<endl;
	  }


	  outF<<"Specfication_Detail: "<<Flag_OutputDetail<<endl;


	if(m_Sim_Type==ballmill)
	{
		    int rev=Revoution_number;

		    if(revflag==0)
		    {
		    	rev--;
		    }
		    else
		    {
		    	rev++;
		    }

		    if(m_KSimulationData->get_InitPosGrid().grid_type==2)
		    {
			    outF<<" RevNum: "<<0<<"  Time: "<<0.0<<" RevComplete: "<<0<<"  RotAngle: "<<Rotation_Angle
							   << "  EParticle: "<< 0.0<<
							   " ELift "<<0.0<<
							   " EDrum: "<<0.0<<
							   " ENorm: "<<0.0<<
							   " EShear: "<<0.0<<endl;
		    }
		    else
		    {
		        outF<<" RevNum: "<<rev<<"  Time: "<<simtime_elapsed<<" RevComplete: "<<revflag<<"  RotAngle: "<<Rotation_Angle
						   << "  EParticle: "<< energy[0]<<
						   " ELift "<<energy[1]<<
						   " EDrum: "<<energy[2]<<
						   " ENorm: "<<energy[3]<<
						   " EShear: "<<energy[4]<<endl;
		    }

	}
	else if(m_Sim_Type==silo)
    {
      outF<<"  Time: "<<simtime_elapsed<<" NumRem: "<<m_Silo_SimData.flow_rate<<endl;
    }
	else if(SimInfo.Rotating_WObject)
	{
		outF<<"  Time: "<<simtime_elapsed<<"  RotSteps: "<<num_WrotSeps<<"  RotAngle: "<<Rotation_Angle<<endl;
	}
    else
    {
      outF<<"  Time: "<<simtime_elapsed<<endl;
    }

	   for ( int i=0; i<SimInfo.Num_Particles; i++ )
	   {

		if (m_d_P_ID[i]!=-1)
		{
		  if(Flag_OutputDetail==1)
		  {
           outF<<m_d_P_ID[i]<<"  "<<m_d_Particle_Type[i]<<"    "<<m_d_Particle_Pos[i].x<<"  "<<m_d_Particle_Pos[i].y<<"  "<<
	        		         m_d_Particle_Pos[i].z<<"    ";

           if(m_sim_particle_type==0)
           {
        	outF<<endl;
           }
           else
           if(m_sim_particle_type==1)
           {
            outF<<m_d_Particle_Quart[i].w<<"  "<<m_d_Particle_Quart[i].x<<
	        		         "  "<<m_d_Particle_Quart[i].y<<"  "<<m_d_Particle_Quart[i].z<<endl;
           }

		  }
		  else
		  if(Flag_OutputDetail==2)
		  {

			  outF<<m_d_P_ID[i]<<"  "<<m_d_Particle_Type[i]<<"    "<<m_d_Particle_Pos[i].x<<"  "<<m_d_Particle_Pos[i].y<<"  "<<
								 m_d_Particle_Pos[i].z<<
								 "    "<<m_d_Particle_Vel[i].x<<"  "<<m_d_Particle_Vel[i].y<<
								 "  "<<m_d_Particle_Vel[i].z;
			   if(m_sim_particle_type==1)
			   {

				outF<< "    "<<m_d_Particle_Quart[i].w<<"   "<<m_d_Particle_Quart[i].x<<
								 "   "<<m_d_Particle_Quart[i].y<< " "<<m_d_Particle_Quart[i].z;
			   }


				   outF<<"       "<<m_d_Particle_RVel[i].x<<"   "<<m_d_Particle_RVel[i].y<<
								 "   "<<m_d_Particle_RVel[i].z<<endl;


		  }
		  else
		  if(Flag_OutputDetail==3)
		  {
			  outF<<m_d_P_ID[i]<<"  "<<m_d_Particle_Type[i]<<"    "<<m_d_Particle_Pos[i].x<<" "<<m_d_Particle_Pos[i].y<<" "<<
							 m_d_Particle_Pos[i].z<<
							 "       "<<m_d_Particle_Vel[i].x<<"   "<<m_d_Particle_Vel[i].y<<
							 "   "<<m_d_Particle_Vel[i].z;

		   if(m_sim_particle_type==1)
		   {
			outF<< "       "<<m_d_Particle_Quart[i].w<<"   "<<m_d_Particle_Quart[i].x<<
							 "   "<<m_d_Particle_Quart[i].y<< " "<<m_d_Particle_Quart[i].z;
		   }

			 outF<<"       "<<m_d_Particle_RVel[i].x<<"   "<<m_d_Particle_RVel[i].y<<
								 "   "<<m_d_Particle_RVel[i].z<<endl;


		  }
		  else if(Flag_OutputDetail==4)
		  {
			  outF<<m_d_P_ID[i]<<"  "<<m_d_Particle_Type[i]<<"    "<<m_d_Particle_Pos[i].x<<" "<<m_d_Particle_Pos[i].y<<" "<<
							 m_d_Particle_Pos[i].z<<"      "<<
							 "       "<<m_d_Particle_Vel[i].x<<"   "<<m_d_Particle_Vel[i].y<<
							 "   "<<m_d_Particle_Vel[i].z;

		   if(m_sim_particle_type==1)
		   {
			outF<< "       "<<m_d_Particle_Quart[i].w<<"   "<<m_d_Particle_Quart[i].x<<
							 "   "<<m_d_Particle_Quart[i].y<< " "<<m_d_Particle_Quart[i].z;
		   }

		   outF<<"       "<<m_d_Particle_RVel[i].x<<"   "<<m_d_Particle_RVel[i].y<<
		   							 "   "<<m_d_Particle_RVel[i].z;

		   outF<<"       "<<length(m_d_Particle_Vel[i]*0.010)<<"   "<<length(m_d_Particle_RVel[i]*0.010)<<"   "<<length(m_d_Particle_Acc[i]*0.010)<<endl;

		  }
		  else if(Flag_OutputDetail==5)
		  {
		  			  outF<<m_d_P_ID[i]<<"  "<<m_d_Particle_Type[i]<<"    "<<m_d_Particle_Pos[i].x<<" "<<m_d_Particle_Pos[i].y<<" "<<
		  							 m_d_Particle_Pos[i].z<<"      ";
		  			  for(int v=0;v<m_KParticleObject[m_d_Particle_Type[i]].num_vertex;v++ )
		  			  {
		  			    float3 point = 	(m_KParticleObject[m_d_Particle_Type[i]].vertex[v] +m_d_Particle_Pos[i]);
		  			    outF<<point.x<<"  "<<point.y<<"  "<<point.z<<"    ";
		  			  }
		  			outF<<endl;
		  }

		  if(m_Sim_Type==silo)
		  {
		   /* Count number left in Silo */
		   if(m_d_Particle_Pos[i].y>m_Silo_SimData.hatch_height)
		   {
			 silo_count++;

             sumV+=length(m_d_Particle_Vel[i]*0.010);

             if(length(m_d_Particle_Vel[i]*0.010)>maxVel)
             {
        	   maxVel=length(m_d_Particle_Vel[i]*0.010);
        	   maxID= m_d_P_ID[i];
             }
	       }
		  }
		  else
		  {
	             sumV+=length(m_d_Particle_Vel[i]*0.010);

	             if(length(m_d_Particle_Vel[i]*0.010)>maxVel)
	             {
	        	   maxVel=length(m_d_Particle_Vel[i]*0.010);
	        	   maxID= m_d_P_ID[i];
	             }
		  }
		}
	   }

      if(m_Sim_Type==silo)
      {
   	   outF<<"# Average Vel(m/s) "<<sumV/silo_count<<endl;
   	   outF<<"# MAXVEL (m/s) "<<maxVel<<" Particle "<<maxID<<endl;
      }
      else
      {
	   outF<<"# Average Vel(m/s) "<<sumV/SimInfo.Num_Particles<<endl;
	   outF<<"# MAXVEL (m/s) "<<maxVel<<" Particle "<<maxID<<endl;
      }

	   if (m_Sim_Type==ballmill)
	   {
	     outF<<"# RevNum: "<<Revoution_number<<"  Time(s): "<<simtime_elapsed<<" Power(W): "<<( ( energy[0] +energy[2])/(m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t))<<endl;
	   }


	   outF.close();



}






/* INPUT:
 * Spec 1: Pos : P_type : P_ID :                   (*poly) A_Pos
 * Spec 2: Pos : P_type : P_ID : Vel               (*poly) A_Pos : AVel
 * Spec 3: Pos : P_type : P_ID : Vel : Acc :       (*poly) A_Pos : AVel
 * Spec 4: Pos : P_type : P_ID : Vel : Acc :       (*poly) A_Pos : AVel: MagV MagR MagA      */
void Read_FromFile(string Fname)
{
	fstream inF;
	string dums;
	char line[256];
    /* Use additional constructor */
	float powers[6];

	int SpecType=-1;
	int SpecTypeR=0;

	powers[0]=0.0f;
	powers[1]=0.0f;
	powers[2]=0.0f;
	powers[3]=0.0f;
	powers[4]=0.0f;
	powers[5]=0.0f;
	powers[6]=0.0f;

	inF.open( ("../Projects/"+ProjectFolder +"/" +Fname ).c_str() );

	getline(inF,dums);


	inF>>dums>>SpecType;

	/* Read Mill simulation */

	if(m_Sim_Type==ballmill)
	{


		int revstate=0;

		inF>>dums>>Revoution_number>>dums>>simtime_elapsed>>dums>>revstate>>dums>>Rotation_Angle>>dums>>powers[0]>>dums>>powers[1]>>dums>>powers[2]
		                                                          >>dums>>powers[3]>>dums>>powers[4];
		powers[0]*=1E4;
		powers[1]*=1E4;
		powers[2]*=1E4;
		powers[3]*=1E4;
		powers[4]*=1E4;


		printf(" Start Angle %f \n",Rotation_Angle);

		m_KDevice->IDevice_Set_Tallys(powers);

		if(simtime_elapsed==0.0 && Fname=="state.tmp")
		{
		  return;
		}
		/* Start from previous state */
		printf("INFO: Reading Mill State from file \n");

			/* Check if Rev was completed */
			if(revstate==0)
			{

				/* We want steps from start of rev */
				float revtime=0.0f;

				if(Revoution_number>=1)
				{
				  /* Completed revs */
				  revtime=(Revoution_number)*m_Mill_SimData.Num_Steps_PerRev*SimInfo.InitalDelta_t;
				}

				/* Get the fraction of steps */
				int numsteps=  ( (simtime_elapsed-revtime) /SimInfo.InitalDelta_t);

				double old= m_Mill_SimData.RadPerStep;

				/* we want steps for that Rev only */

				printf("INFO: Mill Complete  time %f \n",revtime);

				printf("INFO: Mill Elapsed Steps %d \n",numsteps);

				m_Mill_SimData.RadPerStep = m_Mill_SimData.RadPerStep*numsteps;

				if(m_num_KDynamicObjects>0)
				{
				Rotate_VolObject_angle(make_float3(m_KWorldObject[0].cyl_geo.radius,m_KWorldObject[0].cyl_geo.radius,0.0f));


				m_Current_StepNumber= numsteps;

				Draw_DynamicGeometry();

				/* Update the Device */
				m_KDevice->IDevice_UpdateVolObjectPositions(m_KDynamicObject);
				}


				if(SimInfo.Rotating_WObject)
				{
				  Rotate_WObject_angle(make_float3(m_KWorldObject[0].cyl_geo.radius,m_KWorldObject[0].cyl_geo.radius,0.0f));


				m_Current_StepNumber= numsteps;

				//Draw_WorldGeometry();

				/* Update the Device */
				m_KDevice->IDevice_UpdateWorldObjects(m_KWorldObject);
				}


				m_Mill_SimData.RadPerStep=old;

				SimInfo.MillRotVel = -Vel_Mill/100.0;

				Revoution_number++;
			 }

	}
	else if(m_Sim_Type==silo)
	{


		inF>>dums>>simtime_elapsed>>dums>>m_Silo_SimData.num_particles_rem;
		if(simtime_elapsed==0.0 && Fname=="state.tmp")
		{
		  return;
		}
		printf("INFO: Reading Silo State from file \n");

		if( simtime_elapsed > 0.0f )
		{
		 printf("INFO: Time %f \n",simtime_elapsed);
		}

	}
	else if (SimInfo.Rotating_WObject)
	{
		inF>>dums>>simtime_elapsed>>dums>>num_WrotSeps>>dums>>Rotation_Angle;

	    if(simtime_elapsed==0.0 && Fname=="state.tmp")
	    {
		   return;
		}
	    /* Start from previous state */
		printf("INFO: Reading Custom State from file \n");

		/* Update WObject Pos */
		Rotate_WObject_angle(make_float3(m_KWorldObject[0].cyl_geo.radius,m_KWorldObject[0].cyl_geo.radius,0.0f));

		m_KDevice->IDevice_UpdateWorldObjects(m_KWorldObject);

		/* Update DObject Pos */
	    if( m_num_KDynamicObjects > 0 )
		{
	       Rotate_VolObject_angle(make_float3(m_KWorldObject[0].cyl_geo.radius,m_KWorldObject[0].cyl_geo.radius,0.0f));
		   Draw_DynamicGeometry();

		   /* Update the Device */
		   m_KDevice->IDevice_UpdateVolObjectPositions(m_KDynamicObject);
		 }


	}
	else
	{
		inF>>dums>>simtime_elapsed;
		if(simtime_elapsed==0.0 && Fname=="state.tmp")
		{
		  return;
		}
		printf("INFO: Reading State from file \n");
	}

	/* Start from previous state */



	  int dumI;
	  float dumF;

	  for ( int i=0; i<SimInfo.Num_Particles; i++ )
	  {

		/* Read Based on Spectype*/
	    /* Just position */
		if(SpecType==1)
	    {
			inF>>m_d_P_ID[i]>>m_d_Particle_Type[i]>>m_d_Particle_Pos[i].x>>m_d_Particle_Pos[i].y>>m_d_Particle_Pos[i].z;

	    	if(m_sim_particle_type==1)
	    	{
	    	inF>>m_d_Particle_Quart[i].w>>m_d_Particle_Quart[i].x>>m_d_Particle_Quart[i].y
	    		         >>m_d_Particle_Quart[i].z;
	    	}

	    }
	    else
	    if(SpecType==2)
		{
	    	inF>>m_d_P_ID[i]>>m_d_Particle_Type[i]>>m_d_Particle_Pos[i].x>>m_d_Particle_Pos[i].y>>m_d_Particle_Pos[i].z>>
					   m_d_Particle_Vel[i].x>>m_d_Particle_Vel[i].y>>m_d_Particle_Vel[i].z;

			if(m_sim_particle_type==1)
			{
			inF>>m_d_Particle_Quart[i].w>>m_d_Particle_Quart[i].x>>m_d_Particle_Quart[i].y
						 >>m_d_Particle_Quart[i].z;
			}

			inF>>m_d_Particle_RVel[i].x>>m_d_Particle_RVel[i].y>>m_d_Particle_RVel[i].z;

		}
	    else
	    if(SpecType==3 || SpecType==4 )
		{


	    	if(SpecType==3)
	    	{
	    	inF>>m_d_P_ID[i]>>m_d_Particle_Type[i]>>m_d_Particle_Pos[i].x>>m_d_Particle_Pos[i].y>>m_d_Particle_Pos[i].z>>
					   m_d_Particle_Vel[i].x>>m_d_Particle_Vel[i].y>>m_d_Particle_Vel[i].z;
	    	}
	    	else
	    	{
		    	inF>>m_d_P_ID[i]>>m_d_Particle_Type[i]>>m_d_Particle_Pos[i].x>>m_d_Particle_Pos[i].y>>m_d_Particle_Pos[i].z>>
						   m_d_Particle_Vel[i].x>>m_d_Particle_Vel[i].y>>m_d_Particle_Vel[i].z>>
						   m_d_Particle_Acc[i].x>>m_d_Particle_Acc[i].y>>m_d_Particle_Acc[i].z;
	    	}



			if(m_sim_particle_type==1)
			{
			inF>>m_d_Particle_Quart[i].w>>m_d_Particle_Quart[i].x>>m_d_Particle_Quart[i].y
						 >>m_d_Particle_Quart[i].z;
			}

			inF>>m_d_Particle_RVel[i].x>>m_d_Particle_RVel[i].y>>m_d_Particle_RVel[i].z;

			/* SpecType 4 just reads the added magnitude info */
			if(SpecType==4)
			{
			  inF>>dumF>>dumF>>dumF;
			}



			//m_d_Particle_Pos[i]=m_d_Particle_Pos[i]+m_KParticleObject[m_d_Particle_Type[i]].COM;

//	    	if(i<=400)
//	    	{
//	    		m_d_Particle_Type[i] = 0;
//	    	}
//	    	else if(i>400 && i<=600)
//	    	{
//	    		m_d_Particle_Type[i] = 1;
//	    	}
//	    	else if(i>600 && i<=1000)
//	    	{
//	    		m_d_Particle_Type[i] = 0;
//	    	 }
//	    	else if(i>1000 && i<=1200)
//	        {
//	    		m_d_Particle_Type[i] = 1;
//	    	}
//	    	else if(i>1200 && i<=1600)
//	        {
//	    		m_d_Particle_Type[i] = 0;
//	    	}
//	    	else if(i>1600 && i<=1725)
//	        {
//	    		m_d_Particle_Type[i] = 1;
//	    	}
//	    	else
//	    	{
//	    		m_d_Particle_Type[i] = 0;
//	    	}
//
//


		}



	   }
	   inF.close();




	    if(SpecType==1)
	    {
	      /* Set the inital data */
	    	printf("INFO: Setting Positions Only \n");
	        m_KDevice->IDevice_Set_ParticleSystem_Pos(m_d_Particle_Pos,m_d_Particle_Quart,m_d_Particle_Type,m_d_P_ID);
	    }
	    else
		if(SpecType==2)
		{
		      /* Set the inital data */
		    	printf("INFO: Setting Position and Velocity Only \n");
		        m_KDevice->IDevice_Set_ParticleSystem_PosV( m_d_Particle_Pos, m_d_Particle_Quart,
		        		                                   m_d_Particle_Vel,m_d_Particle_RVel,
		        		                                    m_d_Particle_Type,m_d_P_ID );
		}
		else
		if(SpecType==3 || SpecType==4)
		{
			/* Set the inital data */
		  printf("INFO: Setting Full State spectype %d \n",SpecType);
		  m_KDevice->IDevice_Set_ParticleSystem_State( m_d_Particle_Pos,m_d_Particle_Quart, m_d_Particle_Vel,m_d_Particle_RVel,
				                                       m_d_Particle_Acc,

													   m_d_Particle_Type,m_d_P_ID   );

		}
		else
		{
			printf("INFO: Invalid SpecType cannot understand file \n");
			exit(1);
		}

    //outF.close();

    printf("INFO: State Read %d Particles at Time %f \n",SimInfo.Num_Particles,simtime_elapsed);


}

void Write_SurfaceForce()
{
	  m_KDevice->IDevice_Get_W_Forces(m_dc_WallForces,m_dc_WallContacts);

		      stringstream tstep;
		    	tstep<< m_WallForceCount;

		      int num=0;
		      float3 worldOrigin = make_float3(0.0f,0.0f,0.0f);
		      float3 cellSize    = make_float3(1.0f,1.0f,1.0f);

				for (int i=0;i<SimInfo.Num_Particles;i++)
				{
				  if(length(m_dc_WallForces[i])>1.0)
				  {
					  if(i==0)
					  {
					        OFs.open( ("../Results/"+ProjectFolder +"/"+ WorldName + "/" + "SurfaceForces" + tstep.str() + ".dat").c_str() );
						    m_WallForceCount++;
					  }
					  int3 gridPos;

					  gridPos.x = floor(fabs(m_dc_WallContacts[i].x - worldOrigin.x) / cellSize.x);
					  //gridPos.y = floor((WallContacts[i].y - worldOrigin.y) / cellSize.y);
					  gridPos.z = floor(fabs(m_dc_WallContacts[i].z - worldOrigin.z) / cellSize.z);

		//WallContacts[i].x<<" "<<WallContacts[i].z<<"  "<<
				   OFs<<m_dc_WallContacts[i].x<<" "<<m_dc_WallContacts[i].z<<"  "<<length(m_dc_WallForces[i])<<endl;
		           num++;
		          }
		        }
		      OFs<<"# "<<num<<endl;

		      OFs<<endl;
		      OFs<<endl;

		      OFs.close();
}


#endif /* FILEIO_H_ */

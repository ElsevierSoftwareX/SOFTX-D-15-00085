#include "Collision_Detection_Spheres.cuh"
#include "Collision_Detection_Polyhedra.cuh"
#include "../Physics/Collision_Response.cuh"
#include "Integration.cuh"



/*---------------------------------------------------------------------------*/
/* (1) Performs collision detection for objects that have there positions
                         Modified in constant memory                         */
/*---------------------------------------------------------------------------*/
__global__ void VolumeObject_InteractionParticle(
		                                    float3       *dLifterForces,
		                                    float3       *dLifterForces_ang,
		                                    Contact_Info *VObject_Contact_Hist,
		                                    float3       *position_com,
		                                    Quaterion    *position_ornt,
		                                    float3       *velocity_com,
		                                    float3       *velocity_ang,
		                                    float3       *accelrat_com,
		                                    uint         *P_ObjectType,
		                                    int          *P_ID ,
		                                    int           Num_Particles         )
{

	uint index = blockIdx.x*blockDim.x  + threadIdx.x ;


    if( (index < Num_Particles) && P_ID[index]>-1)
	{
      uint d_index=0;

      dLifterForces     [index] = make_float3(0.0f);
      dLifterForces_ang [index] = make_float3(0.0f);

	  CollData  Result_D;

	  Result_D.is_collision  =  false;
	  Result_D.colltype      =  0;
	  Result_D.selected_face = -1;

	  uint   P_type = P_ObjectType  [index];
	  POBJ   PData  = ParticleObject[P_type];
	  float3 Pos_P  = position_com[index];
	  float3 Vel_P  = velocity_com[index];

	  POBJ_ROT POBJ_RT;

		 if(SimParms.particle_type==1)
		 {
		      /* 1. Update Current Orientation of Particle in global cords */
		     POBJ_RT = RotatePolyhedra(P_type, Pos_P,SimParms.Rotation,position_ornt[index] );
		 }



      /* Check only outer particles for mill */
      if( ( DynamicObject[d_index].is_translating || !SimParms.isMillSim || SimParms.Rotating_WObject) || (length(make_float3(Pos_P.x,Pos_P.y,0.0)- SimParms.cent) >
           (SimParms.DObject_CylinderBoundR-2.0f*PData.radius ))                 )
      {



	     /* Loop over all Dynamic Objects */
	     for ( d_index=0; d_index<Num_DynamicObjects; d_index++ )
	     {
		    float3 Pos_D = DynamicObject[d_index].COM;
		    float3 BV    = (Pos_P - Pos_D)*DynamicObject[d_index].Axis;

		    /* Check against bounding cylinder of a DObject */
		    if( length(BV) < ( DynamicObject[d_index].boundR +
		    		           PData.radius) )
		    {

				/* Make local copy Lifter Edges */
			    Edge EdgesD[16];
			    for( uint i=0; i<DynamicObject[ d_index ].num_edges; i++ )
			    {

			      EdgesD[i].Point = DynamicObject[d_index].vertex
			    		 [DynamicObject[d_index].edge[i].point_vindex[0]]
			                                                      ;

			      EdgesD[i].dir = (DynamicObject[ d_index].vertex
			    	  [DynamicObject[ d_index ].edge[i].point_vindex[1]]
			                                  ) - EdgesD[i].Point;

			     }

			     /* Sphere particle */
                 if (SimParms.particle_type==0)
                 {

			        Result_D = Collision_Detection_Sphere_DObject
							( index, d_index ,Pos_P,
							  Vel_P, EdgesD,
							  DynamicObject[d_index].num_edges,PData.radius,VObject_Contact_Hist[index]);

                 }
                 else /* Polyhedral particle */
                 {


					/* 1.1 Make Particle Edges */
					Edge Edges_P[32];
					for( uint i=0; i<ParticleObject[ P_type ].num_edges; i++ )
					{

					 Edges_P[i].Point = POBJ_RT.Vertex[ParticleObject[P_type].edge[i].point_vindex[0]]
														 ;
					 Edges_P[i].dir   = (POBJ_RT.Vertex[ParticleObject[P_type].edge[i].point_vindex[1]]
													) - Edges_P[i].Point;
					}


					 /* 1.Quick check for Face SP  */

					 /* cant use Poly Poly when first object is lifter */
					 bool SP_Check = is_SP_VolumeObject_Poly( DynamicObject[d_index].num_faces,
													     DynamicObject[d_index].faces,
													     Pos_P,
													     ParticleObject[P_type].num_vertex,
													     POBJ_RT.Vertex                      ) ;

					 if(!SP_Check)
					 {
						 SP_Check = is_SP_Poly_VolumeObject(  ParticleObject[P_type].num_faces,
													  POBJ_RT.face,
													  Pos_D,
													  DynamicObject[d_index].num_vertex,
													  DynamicObject[d_index].vertex               ) ;
					 }



					 /* So either there contact or an Edge SP */
					 if( !SP_Check )
					 {

						 Pos_D.z = Pos_P.z;

                          /* Detect Vertex Poly Face Lifter contact */
						  Result_D = Collision_Detection_Polyhedara_VolumeObject_VertexFace
								         ( Pos_P, ParticleObject[P_type].num_vertex, POBJ_RT.Vertex,
								           DynamicObject[d_index].num_faces,  DynamicObject[d_index].faces,
										   DynamicObject[d_index].num_vertex, DynamicObject[d_index].vertex );


					      if ( !Result_D.is_collision )
						  {


							  float3 RelPos = (Pos_D-Pos_P)/length( Pos_D-Pos_P);



							  Result_D = Collision_Detection_DObject_Polyhedra_Edges(
									         DynamicObject[d_index].num_edges, EdgesD,

									         ParticleObject[P_type].num_edges, Edges_P, Pos_P,
										     ParticleObject[P_type].num_faces, POBJ_RT.face,  RelPos);

						  }

					}


                 }

			   if(Result_D.is_collision)
			   {
                   /* No rotation for sphere edge contact */
				   if(SimParms.particle_type==0 && Result_D.colltype==2)
				   {
					   Result_D.contact_point = Pos_P;
				   }

				    if(SimParms.unit_test==1)
				    {
					    printf("PV %d pen dis %f ",P_ID[index],Result_D.dis);
				        PrintVectorD(Vel_P);
				    }

				   Forces Force;
				   float3 Vel_Lifter = make_float3(0.0f);

				   if(DynamicObject[d_index].is_attached_DWorld)
				   {
				     Vel_Lifter = cross( make_float3(0.0,0.0,-SimParms.MillRotVel),
   		                                        Result_D.contact_point -
   		                                        make_float3(WorldObject[0].cyl_geo.radius,
   		            		                    WorldObject[0].cyl_geo.radius,0.0f)       );
				   }
				   else if(DynamicObject[d_index].is_translating)
				   {

					  Vel_Lifter = DynamicObject[d_index].velocity ;
				   }



					  Force = Collision_Response_Particle_VolumeObject( Result_D.Normal,
                                                                 Result_D.dis,
                                                                (Result_D.contact_point-Pos_P),
                                                                 make_float3(0.0f),
                                                                 velocity_ang[index],
                                                                 make_float3(0.0f),
		  					                                     P_type,
		  					                                     Vel_P,
		  						                                 Vel_Lifter,
		  					                                     ParticleObject[P_type].mass    );


				  dLifterForces[index] += Force.trans;

				  /* Rotational Velocity */
				  if( SimParms.Rotation )
				  {
					  dLifterForces_ang[index] += Force.torque;

				  }

					 /* Assume only one lifter at a time */
					 break;
			   }/* End If collision */

		 } /* End Bound Collision */

	   }/* End Loop over Dynamic Objects */

     }/* End loop over lifters bound */

     /* We use contact history so we dont jump from edge to face contact at
      * the corner */

      Contact_Info  local;
      local.obj_id    = d_index;
      local.cont_type = Result_D.colltype;
      local.snum      = Result_D.selected_face;

      VObject_Contact_Hist[index]  = local;

   }


}
/*___________________________________________________________________________*/



/*---------------------------------------------------------------------------*/
/*        (2)        World interaction for Spherical particles               */
/*---------------------------------------------------------------------------*/
__global__ void WorldObject_InteractionParticle( bool      get_surfacePoints,
		                                     bool      is_cylinder_rotation,
		                                     float3    *dWall_Forces,
		                                     float3    *dWall_Forces_ang,
		                                     float3    *dWallContactPoints,
		                                     float3    *position_com,
		                                     Quaterion *position_ornt,
		                                     float3    *velocity_com,
		                                     float3    *velocity_ang,
                                             float3    *accelrat_com,
                                             uint      *P_ObjectType,
                                             int       *P_ID,
                                             int        Num_Particles )
{

   uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

   if( (index < Num_Particles) && P_ID[index]>-1)
   {


	 CollData Result;

	 uint     P_type = P_ObjectType[index];
	 POBJ     PData  = ParticleObject[P_type];
	 float3   Pos_P  = position_com[index];
	 float3   Vel_P  = velocity_com[index];
	 POBJ_ROT POBJ_RT;

	 Result.is_collision    = false;
	 Result.collision_class = UNDEF;




	 dWall_Forces    [index] = make_float3(0.0f);

	 if(SimParms.Rotation)
	 {
	   dWall_Forces_ang[index] = make_float3(0.0f);
	 }

	 if(get_surfacePoints)
	 {
		 dWallContactPoints[index]=make_float3(0.0f);
	 }



	 if(SimParms.particle_type==1)
	 {
	      /* 1. Update Current Orientation of Particle in global cords */
	     POBJ_RT = RotatePolyhedra( P_type, Pos_P,SimParms.Rotation,position_ornt[index] );
	 }



     /* Loop over all World Objects */
	 for ( uint k=0; k<Num_WorldObjects; k++ )
	 {

		 /* If the particle has a very high velocity it can
		  * collied with a surface and its new velocity take
		  * it past another surface */

		if( WorldObject[k].surface_type==plane)
		{
		    /* Check for possible collisions 1 surface at a time */
		    for ( uint i=0; i < WorldObject[k].num_surfaces; i++ )
            {

		     /* Treat as an infinite plane */
		     if(WorldObject[k].surfaces[i].area==-1.0f)
		     {
				 if(SimParms.particle_type==0)
				 {

					 Result = Collision_Detection_Sphere_Plane
								( index, P_type, Pos_P,
								  WorldObject[k].surfaces[i].normal,
								  WorldObject[k].surfaces[i].centroid);

				 }
				 else
				 {
					 printf( "warning no collision method found \n");
                    /*TODO*/
				 }
		     }
			 else
		     {
				 if(SimParms.particle_type==0)
				 {

					 Result = Collision_Detection_Sphere_Surface
								( index, P_type, Pos_P,
								  WorldObject[k].surfaces[i],
								  WorldObject[k].vertex     );

				 }
				 else
				 {
						Result = Collision_Detection_Polyhedra_Surface
								 ( index, P_type, Pos_P, Vel_P,
								   POBJ_RT, WorldObject[k].surfaces[i],
								   WorldObject[k].vertex                );
				 }
		     }
			   if(Result.is_collision)
			   {


			     Forces Force;

				    if(SimParms.unit_test==1)
				    {

				        printf("PS %d pen dis %f ",P_ID[index],Result.dis);
				        PrintVectorD(Vel_P);
				    }

				   if(SimParms.Rotating_WObject && is_cylinder_rotation)
				   {
				     float3 Vel_Lifter = cross( make_float3(0.0,0.0,-SimParms.MillRotVel),
 		                                        Result.contact_point -
 		                                        make_float3(WorldObject[0].cyl_geo.radius,
 		                                        		WorldObject[0].cyl_geo.radius,0.0f)       );


					  Force = Collision_Response_Particle_MovingSurface( Result.Normal,
                                                               Result.dis,
                                                              (Result.contact_point-Pos_P),
                                                               velocity_ang[index],
		  					                                     P_type,
		  					                                     Vel_P,
		  						                                 Vel_Lifter,
		  					                                     ParticleObject[P_type].mass    );
				   }
				   else
				   {
				    Force = Collision_Response_Particle_StaticSurface(
											Vel_P,
											Result.Normal,
											Result.dis,
										   (Result.contact_point -Pos_P),
											velocity_ang[index],
											PData  );
				   }

				   /* Update output array */
				   if(get_surfacePoints)
				   {
				     dWallContactPoints[index] = Result.contact_point;
				   }

					dWall_Forces[index] += Force.trans;

					/* Rotational Velocity */
					 if( SimParms.Rotation )
					 {
						 if(Result.collision_class!=Face_Face)
						 {
						   dWall_Forces_ang[index] += Force.torque;
						 }/* When we have face face particle should stop rotating */
                     }
			   }
            }
		}
	      else if ( WorldObject[k].surface_type == cylinder )
	      {
	         Macro_Cylinder Cyl =  WorldObject[k].cyl_geo;



      	     float3 Velnorm = velocity_com[index];
      	            Velnorm = Velnorm/length(Velnorm);

      	     /* Check particle Cylinder Surface */
      	     if( SimParms.particle_type==0 )
      	     {
      	       Result = Collision_Detection_Sphere_Cylinder
      	    		   ( index,P_type, Pos_P, Vel_P, Cyl);
   	         }
   		     else
   		     {

        	   Result = Collision_Detection_Polyhedra_Cylinder
        	    		   ( index,P_type, Pos_P, Vel_P, Cyl,POBJ_RT);
   		     }


			   if(Result.is_collision)
			   {
			     Forces Force;



			      /* Add Movement of drum */

			      if (is_cylinder_rotation)
			      {


				     float3 Vel_Drum = cross( make_float3(0.0,0.0,-SimParms.MillRotVel),
		                                        Result.contact_point -
		                                        make_float3(WorldObject[0].cyl_geo.radius,
		            		                    WorldObject[0].cyl_geo.radius,0.0f)       );


					  Force = Collision_Response_Particle_MovingSurface( Result.Normal,
                                                              Result.dis,
                                                             (Result.contact_point-Pos_P),
                                                              velocity_ang[index],
		  					                                     P_type,
		  					                                     Vel_P,
		  						                                 Vel_Drum,
		  					                                     ParticleObject[P_type].mass    );
			      }
			      else
			      {

			      Force = Collision_Response_Particle_StaticSurface(
			    		                            Vel_P,
                                                    Result.Normal,
                                                    Result.dis,
                                                    (Result.contact_point -Pos_P),
                                                    velocity_ang[index],
 										            PData   );

			      }


				   /* Update output array */
				   if(get_surfacePoints)
				   {
				     dWallContactPoints[index] = Result.contact_point;
				   }


				   dWall_Forces[index]= Force.trans;

					/* Rotational Velocity if its polyhedra with face flat on cylinder dont induce a moment*/
		           if( SimParms.Rotation )
				   {
		        	 if(!(Result.collision_class==Face_Face && SimParms.particle_type==1))
		        	 {
	                   dWall_Forces_ang[index] += Force.torque;
		        	 }
			       }
			   }


      	     if(Cyl.has_top_cap)
      	     {
          	     if( SimParms.particle_type==0 )
          	     {
 			       Result = Collision_Detection_Sphere_Plane( index,P_type,Pos_P,
 			    		                            Cyl.normal_top_cap,
 			    		                            Cyl.center_top_cap  );
          	     }
          	     else
          	     {
    			   Result = Collision_Detection_Vertcies_Plane( ParticleObject[P_type].num_vertex, POBJ_RT.Vertex,
    						                        Cyl.normal_top_cap,
    												Cyl.center_top_cap       );
          	     }

 			   if(Result.is_collision)
 			   {
 			     Forces Force;

 			       Force = Collision_Response_EndPlate_Particle(
 			    		                            Vel_P,
                                                     Result.Normal,
                                                     Result.dis,
                                                    (Result.contact_point -Pos_P),
                                                     velocity_ang[index],
   										            PData                          );

				   /* Update output array */
				   if(get_surfacePoints)
				   {
				     dWallContactPoints[index] = Result.contact_point;
				   }

				   dWall_Forces[index]= (Force.trans  );


 					/* Rotational Velocity */
 					 if( SimParms.Rotation )
 					 {
 						 dWall_Forces_ang[index] += Force.torque;
 					 }
 			   }
      	     }

      	     if(Cyl.has_bot_cap)
      	     {
          	     if( SimParms.particle_type==0 )
          	     {
 			       Result = Collision_Detection_Sphere_Plane( index,P_type,Pos_P,
 			    		                            Cyl.normal_bot_cap,
 			    		                            Cyl.center_bot_cap  );
          	     }
          	     else
          	     {
    			   Result = Collision_Detection_Vertcies_Plane( ParticleObject[P_type].num_vertex, POBJ_RT.Vertex,
    						                        Cyl.normal_bot_cap,
    												Cyl.center_bot_cap       );
          	     }

 			   if(Result.is_collision)
 			   {
 			     Forces Force;

 			           Force = Collision_Response_EndPlate_Particle(
 			    		                            Vel_P,
                                                     Result.Normal,
                                                     Result.dis,
                                                    (Result.contact_point -Pos_P),
                                                     velocity_ang[index],
   										            PData                          );


 					   /* Update output array */
 					   if(get_surfacePoints)
 					   {

 					     dWallContactPoints[index] =Result.contact_point;
 					   }

 					  dWall_Forces[index]= (Force.trans  );


 					/* Rotational Velocity */
 					 if( SimParms.Rotation )
 					 {
 						 dWall_Forces_ang[index] += Force.torque;

 					  }/* End if rotation */

 			   }/* End if cap collision */

      	     }   /* End bot cap */

	      }/* End cylinder check */



	 }/* End Loop over World Objects */




   } /* thread check */

}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
/*            (3) Particle Particle Collision Response Spheres:
 *                     each particle has its own thread  */
/*---------------------------------------------------------------------------*/
__global__ void Kernel_ParticleInteraction_Spheres_NonSymmetry
							 ( uint      *NumNN,
							   uint      *Broad_List,
							   float3    *position_com,
							   float3    *velocity_com,
							   float3    *velocity_ang,
							   uint      *P_ObjectType,
							   int       *P_ID  ,
							   float3    *force_com,
							   float3    *force_ang,
							   int       Num_Particles)
{
	uint mem_index_A = blockIdx.x*blockDim.x + threadIdx.x;

	/* Check that the mem location is valid and the particle is alive */
	if( (mem_index_A < Num_Particles) && P_ID[mem_index_A]>-1 )
	{

	  /*-----------------------------------------------------------------*/
	                      /* Local data for kernel */
	  CollData Result;
	  float3   relPos;
	  float3   RVel;
	  Forces   Force_PairA;
	 /*-----------------------------------------------------------------*/


	  /* Load information for particle in location A */
	  uint  P_typeA  = P_ObjectType  [mem_index_A];
	  float3 Vel_A    = velocity_com [mem_index_A];
	  float3 Pos_A    = position_com [mem_index_A];
	  float3 VelAng_A = velocity_ang [mem_index_A];

	  /* Translational Force on Particle A */
	  float3 force  = SimParms.Force_Field*ParticleObject[P_typeA].mass;
	  /* Rotational Force on particle A */
	  float3 forceL = make_float3(0.0f);

      /* Loop over all the NN memlocations  of particle A */
	  for ( uint j=0; j< NumNN[mem_index_A]; j++)
	  {
		/* load the memory location of particle who is in a NN */
		int    index_B  = Broad_List[mem_index_A*32 + j];

		/* Load information for particle in location B */
		uint   P_typeB  = P_ObjectType [index_B];
		float3 Pos_B    = position_com [index_B];
        float3 Vel_B    = velocity_com [index_B];
		float3 VelAng_B = velocity_ang [index_B];

		/* Compute local kernel info */
		relPos     = Pos_A - Pos_B  ;
		RVel       = Vel_A - Vel_B ;

		/* Compute Contact information */
		Result.Normal        = relPos/length(relPos); /* Move A away from B */

		Result.dis           = fabs( (length(relPos) -
				(ParticleObject[P_typeA].radius + ParticleObject[P_typeB].radius ) ) ) +0.000010f;


		/* Compute force exerted on Particle A */
		Force_PairA = Collision_Response_Particle_Particle(   RVel,
		          		                                      Result.Normal,
		          		                                      Result.dis,
		          		                                      (-1.0f*Result.Normal*ParticleObject[P_typeA].radius),
		          		                                      (Result.Normal*ParticleObject[P_typeB].radius),
		          		  					                  VelAng_A,
		          		  					                  VelAng_B,
		          		  			  					      P_typeA,
		          		  			  				          P_typeB,
		          		  			  					      Vel_A,
		          		  			  						  Vel_B,
		          		  			  					      ParticleObject[P_typeA].mass,
		          		  			  					      ParticleObject[P_typeB].mass );

//		printf(" %d %d Force %f %f %f \n",P_ID[mem_index_A],P_ID[Broad_List[mem_index_A*32 + j]], Force_PairA.trans.x,
//				                                                Force_PairA.trans.y,Force_PairA.trans.z  );

	    if(SimParms.unit_test==1)
	    {
		  if(P_ID[mem_index_A]==0)
		  {
	        printf("PP pen dis %f ",Result.dis);
	        PrintVectorND(Vel_A);
	        PrintVectorD(Vel_B);
		  }
	    }

         /* Add to force A normal + friction */
         force  += Force_PairA.trans;
         forceL += Force_PairA.torque;
	  }

        /* Update net PP force on A + Gravity */
	      force_com[mem_index_A] = force;

	      /* Update net PP ang force on A */
	      if( SimParms.Rotation )
	      {
	        force_ang[mem_index_A] = forceL;
	      }
	      else
	      {
	    	force_ang[mem_index_A] = make_float3(0.0f);
	      }

	}/* End */

}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*  (4) Particle Particle Collision Response Spheres: uses sparse symmetry  */
/*---------------------------------------------------------------------------*/
__global__ void Kernel_ParticleInteraction_Polyhedra_NonSymmetry
                                             ( uint      *NumNN,
		                                       uint      *Broad_List,
		                                       float3    *position_com,
		                                       Quaterion *position_ornt,
		                                       float3    *velocity_com,
		                                       float3    *velocity_ang,
		                                       float3    *accelration_com,
		                                       uint      *P_ObjectType,
		                                       int       *P_ID  ,
                                               float3    *force_com,
                                               float3    *force_ang,
                                               int       Num_Particles)
{
	uint mem_index = blockIdx.x*blockDim.x + threadIdx.x;

	if( mem_index < Num_Particles && P_ID[mem_index]>-1)
	{

		  Forces Force_PairA;

	      uint  P_typeA   = P_ObjectType     [mem_index];

	      float3 Vel_A    = velocity_com [mem_index];
	      float3 Pos_A    = position_com [mem_index];
	      float3 VelAng_A = velocity_ang [mem_index];

	      POBJ_ROT POBJA;
	      Edge     EdgesA[32];

		  /* Translational Force on Particle A */
			  float3 force  = SimParms.Force_Field*ParticleObject[P_typeA].mass;
		   /* Rotational Force */
			  float3 forceL = make_float3(0.0f);

		  /* Rotate Particle A */

		  if (NumNN[mem_index]>0)
		  {
			 POBJA = RotatePolyhedra( P_typeA, Pos_A,SimParms.Rotation ,position_ornt[mem_index]);

			 /* Make Edges A */
			 for( uint i=0; i < ParticleObject[ P_typeA ].num_edges; i++ )
			 {
					EdgesA[i].Point = POBJA.Vertex[ParticleObject[P_typeA].
													  edge[i].point_vindex[0] ];

					EdgesA[i].dir = (POBJA.Vertex[ParticleObject[P_typeA].
									edge[i].point_vindex[1]] ) - EdgesA[i].Point;


			  }
		   }

		   CollData Result_P;

		   float3 RVel;
		   uint   num_coll = 0;

		   /* Info for NN particle B */
		  int    index_B;
		  uint   P_typeB;

		  float3 Pos_B;
		  float3 Vel_B;
		  float3 VelAng_B;

		  POBJ_ROT POBJB;

		  /* Loop over all the NN of particle A */
		  for ( uint j=0; j< NumNN[mem_index]; j++)
		  {

			  Result_P.is_collision = false;

			  index_B  = Broad_List[mem_index*32 + j];
			  P_typeB  = P_ObjectType[index_B];
			  Vel_B    = velocity_com[index_B];
			  VelAng_B = velocity_ang [index_B];
			  Pos_B    = position_com[index_B];
			  RVel    = Vel_A - Vel_B ;


			   /* Rotate Particle B */
				 POBJB = RotatePolyhedra( P_typeB, Pos_B,SimParms.Rotation,position_ornt[index_B]);

				 Edge EdgesB[32];

				 for( uint i=0; i<ParticleObject[ P_typeB ].num_edges; i++ )
				 {
					EdgesB[i].Point = POBJB.Vertex[ParticleObject[P_typeB].edge[i].
															   point_vindex[0] ];

					EdgesB[i].dir = (POBJB.Vertex[ParticleObject[P_typeB].edge[i].
											 point_vindex[1]]) - EdgesB[i].Point;


				 }

				 float3 RelPos = (Pos_A-Pos_B)/length( Pos_A-Pos_B);


				 /* Check for vertex face SP  AB */
				 bool SP_Check = is_SP_Poly_Poly( ParticleObject[P_typeA].num_faces,
												  POBJA.face, Pos_B,
												  ParticleObject[P_typeB].num_vertex,
												  POBJB.Vertex                        );

				 /* Check for vertex face SP  BA */
				 if(!SP_Check)
				 {
					 SP_Check = is_SP_Poly_Poly( ParticleObject[P_typeB].num_faces,
												 POBJB.face, Pos_A,
												 ParticleObject[P_typeA].num_vertex,
												 POBJA.Vertex                        );
				 }

				 /* Check for vertex face contact or Edge SP */
				 if(!SP_Check)
				 {


                     /* 1. Check Vertex A Face B */
					  Result_P = Collision_Detection_Polyhedara_Polyhedra_VertexFaces
							      ( Pos_A, ParticleObject[P_typeA].num_vertex,
							        POBJA.Vertex,
							        Pos_B ,ParticleObject[P_typeB].num_faces,
							        POBJB.face, ParticleObject[P_typeB].num_vertex,
							        POBJB.Vertex, ParticleObject[P_typeB],
							        P_typeB,mem_index,index_B);

					  /* 2. Check Vertex B Face A */
					  if (!Result_P.is_collision)
					  {

						  Result_P = Collision_Detection_Polyhedara_Polyhedra_VertexFaces
								  ( Pos_B, ParticleObject[P_typeB].num_vertex, POBJB.Vertex,
                                   Pos_A ,ParticleObject[P_typeA].num_faces,  POBJA.face,
					                ParticleObject[P_typeA].num_vertex, POBJA.Vertex,
					                ParticleObject[P_typeA], P_typeB, mem_index, index_B  );
						  /* B-A so reverse normal */
						  Result_P.Normal *= -1.0f;
					  }


					  /* Check Edges */
					  if (!Result_P.is_collision)
					  {

						  Result_P = Collision_Detection_Polyhedra_Polyhedra_Edges(EdgesA, EdgesB,
							     			 ParticleObject[P_typeA].num_edges,
							     			 ParticleObject[P_typeB].num_edges,
							                  RelPos,Pos_B,ParticleObject[P_typeB].num_faces,POBJB.face);




					      if(Result_P.is_collision)
						  {
	  						 Result_P.collision_class = Edge_Edge;

	  						 /* Limit max pen distance for Edge Edge contact */
	  						 Result_P.dis= min(0.025f*ParticleObject[P_typeA].radius,Result_P.dis);

							 Result_P.colltype = 2;
						   }
						   else
						   {
	                          Result_P.is_collision=false;
							  Result_P.collision_class = SP_PLANE;
						   }
					  }
                   }





				  if(Result_P.is_collision)
				  {

					/* Indicates a contact error */
					  if( Result_P.dis> 0.1500f*ParticleObject[P_typeA].radius)
					  {
			  			  printf("%f !!warning large dis  Coll type %d Coll claf %d : P %d %d  vel %f  || ",
			  					  Result_P.dis,Result_P.colltype, Result_P.collision_class, P_ID[mem_index], P_ID[index_B], length((velocity_com[mem_index])/100.0f));

						   Result_P.Normal = RelPos/length(RelPos);
						   Result_P.dis = 0.025f*ParticleObject[P_typeA].radius;
						   Result_P.contact_point = Pos_A;
					  }

					  /* Limits velocity in case of large penetrations */
					  if( Result_P.dis > 0.0500f*ParticleObject[P_typeA].radius)
					  {
						   Result_P.Normal = RelPos/length(RelPos);
						   if(length(RVel)>50.0)
						   {
							   Result_P.dis = 0.010f*ParticleObject[P_typeA].radius;
						   }
						   else if(length(RVel)>150.0)
						   {
							   Result_P.dis = 0.015f*ParticleObject[P_typeA].radius;
						   }
						   else if(length(RVel)>250.0)
						   {
							   Result_P.dis = 0.020f*ParticleObject[P_typeA].radius;
						   }
						   else
						   {
							   Result_P.dis = 0.0030f*ParticleObject[P_typeA].radius;
						   }

						   Result_P.contact_point = Pos_A;
					  }


				      Force_PairA = Collision_Response_Particle_Particle(   RVel,
						          		                                      Result_P.Normal,
						          		                                      Result_P.dis,
						          		                                      Result_P.contact_point -Pos_A,
						          		                                      Result_P.contact_point -Pos_B,
						          		  					                  VelAng_A,
						          		  					                  VelAng_B,
						          		  			  					      P_typeA,
						          		  			  				          P_typeB,
						          		  			  					      Vel_A,
						          		  			  						  Vel_B,
						          		  			  					      ParticleObject[P_typeA].mass,
						          		  			  					      ParticleObject[P_typeB].mass );


			     /* For particle A we can sum contributions in local memory */
			      force  += Force_PairA.trans;/* normal + tangential */
			      forceL += Force_PairA.torque;/* Rotational */

			      num_coll++; /* Increment number of collisions */

				}/* End if collision */

			  }/* End checking neighbours of A */


	      force_com[mem_index] = force;

	      if( SimParms.Rotation )
	      {
	        force_ang[mem_index] = forceL;
	      }
	      else
	      {
	    	force_ang[mem_index] = make_float3(0.0f);
	      }

	}/* End */

}
/*---------------------------------------------------------------------------*/





/*___________________________________________________________________________*/
/*  (4) TEST NEED TO DO with Mem Adress  */
/*___________________________________________________________________________*/
__global__ void Kernel_ParticleInteraction_Polyhedra_NonSymmetryHist
                                             ( uint           *NumNN,
		                                       uint           *Broad_List,
		                                       float3         *position_com,
		                                       Quaterion      *position_ornt,
		                                       float3         *velocity_com,
		                                       float3         *velocity_ang,
		                                       float3         *accelration_com,
		                                       uint           *P_ObjectType,
		                                       int            *P_ID  ,
                                               float3         *force_com,
                                               float3         *force_ang,
                                               Contact_InfoPP *PP_Hist,
                                               int            *PP_Hist_Num,
                                               int             Num_Particles  )
{
	uint mem_index = blockIdx.x*blockDim.x + threadIdx.x;


	if( mem_index < Num_Particles && P_ID[mem_index]>-1)
	{
          bool is_oldContact = false;
		  CollData Result_P;


		  Forces Force_PairA;
	      uint  P_typeA   = P_ObjectType [mem_index];
	      float MassA     = ParticleObject[P_typeA].mass;

	      float3 Vel_A    = velocity_com [mem_index];
	      float3 Pos_A    = position_com [mem_index];
	      float3 VelAng_A = velocity_ang [mem_index];

	      POBJ_ROT POBJA;
	      Edge     EdgesA[32];

		  /* Translational Force on Particle A */
			  float3 force  = SimParms.Force_Field*MassA;
		   /* Rotational Force */
			  float3 forceL = make_float3(0.0f);

		  /* Rotate Particle A */
		  if ( NumNN[mem_index] > 0 )
		  {
			 POBJA = RotatePolyhedra(P_typeA, Pos_A,SimParms.Rotation ,position_ornt[mem_index]);

			 /* Make Edges A */
			 for( uint i=0; i < ParticleObject[ P_typeA ].num_edges; i++ )
			 {
					EdgesA[i].Point = POBJA.Vertex[ParticleObject[P_typeA].
													  edge[i].point_vindex[0] ];

					EdgesA[i].dir = (POBJA.Vertex[ParticleObject[P_typeA].
									edge[i].point_vindex[1]] ) - EdgesA[i].Point;
			  }
		   }



		   float3 RVel;
		   uint   num_coll = 0;

		   /* Info for NN particle B */
		  int      mem_index_B;
		  uint     P_typeB;
		  int      PID_B;
		  float3   Pos_B;
		  float3   Vel_B;
          float3   VelAng_B;
		  POBJ_ROT POBJB;

		  Contact_InfoPP contact_hist;
		  bool           SP_Check;

		  if (NumNN[mem_index]>30)
		  {
			  printf("!!!warning exceeded MAX HIST\n");
		  }

		  /* Loop over all the NN of particle A */
		  for ( uint j=0; j< NumNN[mem_index]; j++ )
		  {

			  Result_P.is_collision = false;
			  Result_P.colltype     = -1;
              /* Memory index of neighbor : Sort ensures a better cache fetch ratio */
			  mem_index_B  = Broad_List[mem_index*32 + j];
			  P_typeB      = P_ObjectType[mem_index_B];

			  Vel_B        = velocity_com[mem_index_B];
			  VelAng_B     = velocity_ang[mem_index_B];
			  Pos_B        = position_com[mem_index_B];
			  RVel         = Vel_A - Vel_B ;
              PID_B        = P_ID[mem_index_B];

			   /* Rotate Particle B */
				 POBJB = RotatePolyhedra(P_typeB, Pos_B,SimParms.Rotation,position_ornt[mem_index_B]);

				 Edge EdgesB[32];

				 for( uint i=0; i<ParticleObject[ P_typeB ].num_edges; i++ )
				 {
					EdgesB[i].Point = POBJB.Vertex[ParticleObject[P_typeB].edge[i].
															   point_vindex[0] ];

					EdgesB[i].dir = (POBJB.Vertex[ParticleObject[P_typeB].edge[i].
											 point_vindex[1]]) - EdgesB[i].Point;


				 }

				 float3 RelPos = (Pos_A-Pos_B)/length( Pos_A-Pos_B);


				 /* Check if there is an existing contact */
				 for (int i=0; i<PP_Hist_Num[mem_index]; i++)
				 {
				   if ( PP_Hist[mem_index*32 + i].obj_id==PID_B )
				   {
					 is_oldContact = true;
					 contact_hist = PP_Hist[mem_index*32 + i];
					 break;
				   }
				 }


				 if (is_oldContact)
				 {
				     if(contact_hist.cont_type==SP_PLANE)
				     {
				       if(contact_hist.AorB==0)
				       {
				         SP_Check = is_SP_Face(POBJA.face[contact_hist.contact_face],Pos_B,
													  ParticleObject[P_typeB].num_vertex,
													  POBJB.Vertex);
				       }
				       else
				       {
					     SP_Check = is_SP_Face(POBJB.face[contact_hist.contact_face],Pos_A,
														  ParticleObject[P_typeA].num_vertex,
														  POBJA.Vertex);
				       }
				     }

				    /* Check for vertex Face Contact */
				    if( !SP_Check || contact_hist.cont_type>1)
				    {
				    	/* Face A */
				    	if(contact_hist.AorB==0)
				    	{
				    	  Result_P = Collision_Detection_Vertcies_Face( ParticleObject[P_typeB].num_vertex,
								  POBJB.Vertex,POBJA.Vertex,POBJA.face[contact_hist.contact_face],
								  ParticleObject[P_typeA].face[contact_hist.contact_face]);
				    	}
				    	else if(contact_hist.AorB==1)
				    	{
				    		Result_P = Collision_Detection_Vertcies_Face( ParticleObject[P_typeA].num_vertex,
								  POBJA.Vertex, POBJB.Vertex, POBJB.face[contact_hist.contact_face],
								  ParticleObject[P_typeB].face[contact_hist.contact_face]);
				    	}
				    }
				    else if (contact_hist.cont_type==2)
				    {
						  Result_P = Collision_Detection_Polyhedra_Polyhedra_Edges(EdgesA, EdgesB,
											 ParticleObject[P_typeA].num_edges,
											 ParticleObject[P_typeB].num_edges,
											  RelPos,Pos_B,ParticleObject[P_typeB].num_faces,POBJB.face);
						  if (Result_P.is_collision)
						  {
							  Result_P.dis= min(0.025f*ParticleObject[P_typeA].radius,Result_P.dis);
						  }

				    }

				 }/* Do a new search for contact info */
				 else if ( !is_oldContact || (!SP_Check || !Result_P.is_collision) )
				 {
					 /* Check for vertex face SP  AB */
					 SP_Check = is_SP_Poly_Poly( ParticleObject[P_typeA].num_faces,
													  POBJA.face, Pos_B,
													  ParticleObject[P_typeB].num_vertex,
													  POBJB.Vertex, &PP_Hist[mem_index].contact_face);

					 /* Check for vertex face SP  BA */
					 if(!SP_Check)
					 {
						 SP_Check = is_SP_Poly_Poly( ParticleObject[P_typeB].num_faces,
													 POBJB.face, Pos_A,
													 ParticleObject[P_typeA].num_vertex,
													 POBJA.Vertex,&PP_Hist[mem_index].contact_face);
					 }
					 else
					 {
						 contact_hist.cont_type = SP_PLANE; /* We have a SP Face A Vertex B */
						 contact_hist.AorB      = 0;
						 contact_hist.obj_id    = P_ID[mem_index_B];/* Store NN ID */
					 }


				     /* Check for vertex face contact or Edge SP */
				     if(!SP_Check)
				     {

                         /* 1. Check Vertex A Face B */
					     Result_P = Collision_Detection_Polyhedara_Polyhedra_VertexFaces
							      ( Pos_A, ParticleObject[P_typeA].num_vertex,
							        POBJA.Vertex,
							        Pos_B ,ParticleObject[P_typeB].num_faces,
							        POBJB.face, ParticleObject[P_typeB].num_vertex,
							        POBJB.Vertex, ParticleObject[P_typeB],
							        P_typeB,mem_index,mem_index_B, &contact_hist.contact_face );

					      /* 2. Check Vertex B Face A */
					      if (!Result_P.is_collision)
					      {

						     Result_P = Collision_Detection_Polyhedara_Polyhedra_VertexFaces
								  ( Pos_B, ParticleObject[P_typeB].num_vertex, POBJB.Vertex,
                                    Pos_A ,ParticleObject[P_typeA].num_faces,  POBJA.face,
					                ParticleObject[P_typeA].num_vertex, POBJA.Vertex,
					                ParticleObject[P_typeA], P_typeB, mem_index, mem_index_B, &contact_hist.contact_face   );
						      /* B-A so reverse normal */
						     Result_P.Normal *= -1.0f;
					      }
					      else
					      {
						    contact_hist.cont_type = Vertex_Face; /* Vertex A Face B */
						    contact_hist.AorB = 1;
						    contact_hist.obj_id = P_ID[mem_index_B];/* Store NN ID */
					      }

					      /* Check Edges */
					      if (!Result_P.is_collision)
					      {

						    Result_P = Collision_Detection_Polyhedra_Polyhedra_Edges(EdgesA, EdgesB,
											 ParticleObject[P_typeA].num_edges,
											 ParticleObject[P_typeB].num_edges,
											  RelPos,Pos_B,ParticleObject[P_typeB].num_faces,POBJB.face);


						    if (!Result_P.is_collision)
						    {
						      contact_hist.cont_type = SP_PLANE; /* We have a SP Edges A and B */
						      contact_hist.AorB      = 2;
						      contact_hist.obj_id     = P_ID[mem_index_B]; /* Store NN ID */
						    }
						    else
						    {
							  Result_P.dis= min(0.025f*ParticleObject[P_typeA].radius,Result_P.dis);
							  contact_hist.cont_type = Edge_Edge; /* Edge contact */
							  contact_hist.obj_id    = P_ID[mem_index_B];/* Store NN ID */
						     }
                          }
					      else
					      {
						     contact_hist.cont_type = Vertex_Face; /* Vertex B Face A */
						     contact_hist.AorB      = 1;
						     contact_hist.obj_id     = P_ID[mem_index_B];/* Store NN ID */
					     }
				   }
				   else
				   {
					 contact_hist.cont_type = SP_PLANE; /* We have a SP Face B Vertex A */
					 contact_hist.AorB = 1;
					 contact_hist.obj_id = P_ID[mem_index_B]; /* Store NN ID */
				   }

			} /* End Finding new contact */

			/* If this is a new contact update History List */
			if(!is_oldContact)
			{
			  PP_Hist[mem_index*32 + j ] = contact_hist;
			}

			if(Result_P.is_collision)
		    {

						Force_PairA = Collision_Response_Particle_Particle(   RVel,
						          		                                      Result_P.Normal,
						          		                                      Result_P.dis,
						          		                                      Result_P.contact_point -Pos_A,
						          		                                      Result_P.contact_point -Pos_B,
						          		  					                  VelAng_A,
						          		  					                  VelAng_B,
						          		  			  					      P_typeA,
						          		  			  				          P_typeB,
						          		  			  					      Vel_A,
						          		  			  						  Vel_B,
						          		  			  					      ParticleObject[P_typeA].mass,
						          		  			  					      ParticleObject[P_typeB].mass );

						if ( P_ID[mem_index]==0 && Result_P.dis > ParticleObject[P_typeA].radius*0.10f)
						{
							printf("warning max pen dis of %f exceeded %f \n", ParticleObject[P_typeA].radius*0.10f,Result_P.dis );
						}
			     /* For particle A we can sum contributions in local memory */
			      force  += Force_PairA.trans;/* normal + tangential */
			      forceL += Force_PairA.torque;/* Rotational */

			      num_coll++; /* Increment number of collisions */

			}/* End if collision */





			  }/* End checking neighbours of A */

		  /* Update number of current NN */
		  PP_Hist_Num[mem_index] = NumNN[mem_index];

	      force_com[mem_index] = force;

	      if( SimParms.Rotation )
	      {
	        force_ang[mem_index] = forceL;
	      }
	      else
	      {
	    	force_ang[mem_index] = make_float3(0.0f);
	      }

	}/* End */

}
/*---------------------------------------------------------------------------*/





/* TODO: Future Development methods */

/*---------------------------------------------------------------------------*/
/*       (TESTING) Particle Particle Collision Response Spheres: uses symmetry     */
/*---------------------------------------------------------------------------*/
__global__ void Kernel_ParticleInteraction_Spheres_Symmetry
                                             ( uint   *NumNN,
		                                       uint   *Broad_List,

		                                       float  *force_com_X,
		                                       float  *force_com_Y,
		                                       float  *force_com_Z,

		                                       float  *force_ang_X,
		                                       float  *force_ang_Y,
		                                       float  *force_ang_Z,

		                                       float3 *position_com,
		                                       float3 *velocity_com,
		                                       float3 *velocity_ang,
		                                       uint   *P_ObjectType,
		                                       int    *P_ID,
		        							   int     Num_Particles )
{
	uint mem_index_A = blockIdx.x*blockDim.x + threadIdx.x;

	/* Check that the mem location is valid and the particle is alive */
	if( (mem_index_A < Num_Particles) && P_ID[mem_index_A]>-1 )
	{

	  /*-----------------------------------------------------------------*/
	                      /* Local data for kernel */
	  CollData Result;

	  float3  relPos;
	  float3  RVel;
	  Forces Force_PairA;
	  int num_coll=0;
	 /*-----------------------------------------------------------------*/


	  /* Load information for particle in location A */
	  uint  P_typeA  = P_ObjectType  [mem_index_A];
	  float3 Vel_A    = velocity_com [mem_index_A];
	  float3 Pos_A    = position_com [mem_index_A];
	  float3 VelAng_A = velocity_ang [mem_index_A];


	  /* Translational Force on Particle A */
	  float3 force  = SimParms.Force_Field*ParticleObject[P_typeA].mass;
	  /* Rotational Force on particle A */
	  float3 forceL = make_float3(0.0f);

      /* Loop over all the NN memlocations  of particle A */
	  for ( uint j=0; j< NumNN[mem_index_A]; j++)
	  {
		/* load the memory location of particle who is in a NN */
		int    mem_index_B  = Broad_List[mem_index_A*32 + j];

		/* Load information for particle in location B */
		uint   P_typeB  = P_ObjectType [mem_index_B];
		float3 Pos_B    = position_com [mem_index_B];
        float3 Vel_B    = velocity_com [mem_index_B];
		float3 VelAng_B = velocity_ang [mem_index_B];

		/* Compute local kernel info */
		relPos     = Pos_A - Pos_B  ;
		RVel       = Vel_A - Vel_B ;

		/* Compute Contact information */
		Result.Normal        = relPos/length(relPos); /* Move A away from B */

		Result.dis           = fabs( (length(relPos) -
				(ParticleObject[P_typeA].radius + ParticleObject[P_typeB].radius ) ) ) +0.000010f;


		/* Compute force exerted on Particle A */
		Force_PairA = Collision_Response_Particle_Particle(   RVel,
		          		                                      Result.Normal,
		          		                                      Result.dis,
		          		                                      (-1.0f*Result.Normal*ParticleObject[P_typeA].radius),
		          		                                      (Result.Normal*ParticleObject[P_typeB].radius),
		          		  					                  VelAng_A,
		          		  					                  VelAng_B,
		          		  			  					      P_typeA,
		          		  			  				          P_typeB,
		          		  			  					      Vel_A,
		          		  			  						  Vel_B,
		          		  			  					      ParticleObject[P_typeA].mass,
		          		  			  					      ParticleObject[P_typeB].mass );

		printf(" %d %d Force %f %f %f \n",P_ID[mem_index_A],P_ID[Broad_List[mem_index_A*32 + j]], Force_PairA.trans.x,
				                                                Force_PairA.trans.y,Force_PairA.trans.z  );

         /* Add to force A normal + friction */
         force  += Force_PairA.trans;
         forceL += Force_PairA.torque;

          /* Add translational forces to particle B */
          atomicAdd(&force_com_X[mem_index_B], -Force_PairA.trans.x );
          atomicAdd(&force_com_Y[mem_index_B], -Force_PairA.trans.y );
          atomicAdd(&force_com_Z[mem_index_B], -Force_PairA.trans.z );


          if(SimParms.Rotation)
          {
            /* Add rotational forces to particle B */
            atomicAdd(&force_ang_X[mem_index_B], -Force_PairA.torque.x );
            atomicAdd(&force_ang_Y[mem_index_B], -Force_PairA.torque.y );
            atomicAdd(&force_ang_Z[mem_index_B], -Force_PairA.torque.z );
          }


	      num_coll++;
	   }


	    /* Update particle A */
	  atomicAdd(&force_com_X[mem_index_A], force.x );
      atomicAdd(&force_com_Y[mem_index_A], force.y );
      atomicAdd(&force_com_Z[mem_index_A], force.z );

      if(SimParms.Rotation && num_coll>0 )
      {
        /* Update rotational Force A */
        atomicAdd(&force_ang_X[mem_index_A], forceL.x );
        atomicAdd(&force_ang_Y[mem_index_A], forceL.y );
        atomicAdd(&force_ang_Z[mem_index_A], forceL.z );
      }


	}/* End */

}
/*___________________________________________________________________________*/





/*___________________________________________________________________________*/
/*  (TESTING) Particle Particle Collision Response Spheres: uses sparse 2D Matrix  */
/*___________________________________________________________________________*/
__global__ void Kernel_ParticleInteractionPoly_Symmetry(
		                                       uint  *NumNN,
		                                       uint  *Broad_List,
		                                       float *force_com_X,
		                                       float *force_com_Y,
		                                       float *force_com_Z,
		                                       float *force_ang_X,
		                                       float *force_ang_Y,
		                                       float *force_ang_Z,

		                                       float3    *position_com,
		                                       Quaterion *position_ornt,
		                                       float3    *velocity_com,
		                                       float3    *velocity_ang,
		                                       uint      *P_ObjectType,
		                                       int       *P_ID,
		                                       int       Num_Particles)
{

}
/*___________________________________________________________________________*/






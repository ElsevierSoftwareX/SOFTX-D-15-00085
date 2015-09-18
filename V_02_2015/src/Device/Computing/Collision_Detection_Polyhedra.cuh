/* govender.nicolin@gmail.com */

/*---------------------------------------------------------------------------*/
/*            (1) Checks if there is Vertex Plane contact                    */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Vertcies_Plane( uint     num_vertex,
		                                                float3  *Vertex_List,
		                                                float3   plane_normal,
		                                                float3   plane_centroid )
{
	CollData Result;

	Result.contact_point = make_float3(0.0f); /* Point of contact */

	float  vertex_dis;    /* _|_ distance to the surface*/

    int    num_vertexCount  = 0;
	float  dis_max          = 0.0; /* Max variable */


     /*  Loop  over all vertex of the particle */
     for( uint i=0; i< num_vertex; i++ )
     {
	    /* 5.1) Now get the _|_ intersection distance EQ 1*/
        vertex_dis = dot( plane_normal, ( Vertex_List[i]  - plane_centroid ) );

        /* Check if EQ1 < 0 */
        if( vertex_dis < 0.000000f )
        {
           num_vertexCount++;

           Result.contact_point  += Vertex_List[i] ; /* Average surface point */

           /* get largest  vertex penetration */
           if ( ( fabs(vertex_dis) > dis_max) )
           {
  		     dis_max     = fabs(vertex_dis);
  		   }

	    }

    }/* End all vertex */


    /* No Collision Exit */
    if( num_vertexCount==0 )
    {
      Result.is_collision    = false;
      Result.collision_class = SP_PLANE;
      return Result;
    }

    /* We have contact */
    Result.is_collision    = true;


    if( num_vertexCount==1 )
    {
        Result.collision_class = Vertex_Face;
    }
    if (num_vertexCount==2 )
    {
    	Result.collision_class = Edge_Face;
    }
    else
    {
    	Result.collision_class = Face_Face;
    }

    Result.contact_point /= (float)num_vertexCount;
    Result.Normal         =  plane_normal;
	Result.dis            =  dis_max;

    return Result;
}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
/*    (2)        Checks if there is Vertex Surface Contact
 *               *NB: check tol for highly faceted objects                   */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Vertcies_Surface( uint     num_vertex,
		                                                  float3  *Vertex_List,
		                                                  float3  *surface_vertex,
		                                                  Surface  surface         )
{
	CollData Result;
	Result.is_collision  = false;
	Result.contact_point = make_float3(0.0f);

	float3  hit_point;   /* plane intersection point */
	float   vertex_dis;  /* _|_ distance to the surface*/

	float dis_max        = 0.0; /* Max variable CM */
    uint  num_vertexCount = 0;

    for( uint i=0; i<num_vertex; i++ ) /* Loop over Particle vertex */
    {

	    /* distance between vertex and surface */
        vertex_dis = dot( surface.normal, ( Vertex_List[i] - surface.centroid ) );

        /* if this is negative then possible contact */
        if( vertex_dis < 0.000000f )
        {

            /* Ray Trace point to Surface Plane normal points out of surface */
        	hit_point  = Vertex_List[i] + fabs(vertex_dis)*surface.normal;

        	/* Check if the point is on the surface TOL= 1% */
        	if( is_PointOnSurface( surface.num_vertex, surface_vertex,
        	                       surface.vertex_Order, hit_point,
        	                              surface.area,surface.Tol)     )
        	{


                /* Increment contact points */
                num_vertexCount++;

                /* Average contact points */
                Result.contact_point  += hit_point;

                /* Get the largest  vertex penetration */
                if ( ( fabs(vertex_dis) > dis_max ) )
                {
                   Result.is_collision   = true;
  		           dis_max               = fabs(vertex_dis);
  		        }

                /* face face contact so no moment */
                if( num_vertexCount > 2 )
                {

                  Result.collision_class = Face_Face;
                  Result.Normal          = surface.normal;
                  Result.dis             = dis_max;
                  /* Contact point world coordinates */
                  Result.contact_point   = make_float3(0.0f);

                  return Result;
                }
        	}

	    }/* End vertex plane penetration */

    }/* End loop over all vertex */

     /* If there is a SP then NO contact */
     if( Result.is_collision == false )
     {
       Result.collision_class = SP_PLANE;
       return Result;
     }

       Result.is_collision   = true;

       if( num_vertexCount==1 )
       {
           Result.collision_class = Vertex_Face;
       }
       else
       {
       	Result.collision_class = Edge_Face;
       }

       Result.Normal         = surface.normal;
       Result.dis            = dis_max;

       /* Contact point in world coordinates */
       Result.contact_point /= ((float)num_vertexCount);

       return Result;

}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*    (2)        Checks if there is Vertex Surface Contact
 *               *NB: check tol for highly faceted objects                   */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Vertcies_Face( uint     num_vertex,
		                                                  float3  *Vertex_List,
		                                                  float3  *surface_vertex,
		                                                  Plane  surface,
		                                                  Surface Face)
{
	CollData Result;
	Result.is_collision  = false;
	Result.contact_point = make_float3(0.0f);

	float3  hit_point;   /* plane intersection point */
	float   vertex_dis;  /* _|_ distance to the surface*/

	float dis_max        = 0.0; /* Max variable CM */
    uint  num_vertexCount = 0;

    for( uint i=0; i<num_vertex; i++ ) /* Loop over Particle vertex */
    {

	    /* distance between vertex and surface */
        vertex_dis = dot( surface.normal, ( Vertex_List[i] - surface.centroid ) );

        /* if this is negative then possible contact */
        if( vertex_dis < 0.000000f )
        {

            /* Ray Trace point to Surface Plane normal points out of surface */
        	hit_point  = Vertex_List[i] + fabs(vertex_dis)*surface.normal;

        	/* Check if the point is on the surface TOL= 1% */
        	if( is_PointOnSurface( Face.num_vertex, surface_vertex,
        			               Face.vertex_Order, hit_point,
        			               Face.area,Face.Tol)     )
        	{


                /* Increment contact points */
                num_vertexCount++;

                /* Average contact points */
                Result.contact_point  += hit_point;

                /* Get the largest  vertex penetration */
                if ( ( fabs(vertex_dis) > dis_max ) )
                {
                   Result.is_collision   = true;
  		           dis_max               = fabs(vertex_dis);
  		        }

                /* face face contact so no moment */
                if( num_vertexCount > 2 )
                {

                  Result.collision_class = Face_Face;
                  Result.Normal          = surface.normal;
                  Result.dis             = dis_max;
                  /* Contact point world coordinates */
                  Result.contact_point   = make_float3(0.0f);

                  return Result;
                }
        	}

	    }/* End vertex plane penetration */

    }/* End loop over all vertex */

     /* If there is a SP then NO contact */
     if( Result.is_collision == false )
     {
       Result.collision_class = SP_PLANE;
       return Result;
     }

       Result.is_collision   = true;

       if( num_vertexCount==1 )
       {
           Result.collision_class = Vertex_Face;
       }
       else
       {
       	Result.collision_class = Edge_Face;
       }

       Result.Normal         = surface.normal;
       Result.dis            = dis_max;

       /* Contact point in world coordinates */
       Result.contact_point /= ((float)num_vertexCount);

       return Result;

}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*   (3) Returns the Closest 3 faces to a vertex inside a polyhedra
 *               bool is_point inside                                        */
/*---------------------------------------------------------------------------*/
__device__ PointInPoly Collision_Detection_Vertex_InPoly_3Faces( float3  Pos_P,
		                                                         float3  P_vertex,
                                                                 uint    num_faces,
                                                                 Plane  *Faces,
                                                                 float3  Pos_Faces )
{

      bool   is_inside_plane;
	  float  vertex_dis ;

	  PointInPoly res;

      res.is_point_inside = false;

	  res.Close_Face_num[0] = -1;
	  res.Close_Face_num[1] = -1;
	  res.Close_Face_num[2] = -1;

	  res.distance[0] = 10.0f;/* should use pdata.bradius*2 */
	  res.distance[1] = 10.0f;
	  res.distance[2] = 10.0f;

      /* Loop over Planes of Particle A */
	  for ( uint i=0; i< num_faces; i++)
	  {
         is_inside_plane = false;

         /* _|_ distance between vertex and face */
         vertex_dis = dot(Faces[i].normal,( P_vertex - Faces[i].centroid ));

         /* if we past the plane report collision */
	     if( vertex_dis < -0.0000100f )
	     {
	        is_inside_plane     = true; /* point inside*/

	        /* We only want faces that the COM of the particle is on the correct side of */
	        if( dot( Faces[i].normal,( Pos_P - Faces[i].centroid ) ) > 0.0 )
	        {
	            if(fabs( vertex_dis) < res.distance[0] )
	            {
	               /* update the other 2 distances */
	               if( res.Close_Face_num[0] > -1 )
	               {
		              if( res.Close_Face_num[1] >-1 )
	                  {
		            	 res.distance  [2] = res.distance  [1];
		            	 res.Close_Face_num[2] = res.Close_Face_num[1];
	                  }
	            	  res.distance  [1] = res.distance  [0];
	            	  res.Close_Face_num[1] = res.Close_Face_num[0];
	               }

	                res.distance  [0]   = fabs(vertex_dis);
	            	res.Close_Face_num[0] = i;

	              }
	              else if( fabs(vertex_dis) < res.distance[1] )
	              {
	            	  /* update 3rd close face */
	            	  if( res.Close_Face_num[1] > -1 )
                      {
	            	     res.distance  [2] = res.distance  [1];
	            	 	 res.Close_Face_num[2] = res.Close_Face_num[1];
                      }

	            	  res.distance  [1] = fabs(vertex_dis);
	            	  res.Close_Face_num[1] = i;
	              }
	              else if( fabs(vertex_dis) < res.distance[2] )
	              {
	            	  res.distance  [2] = fabs(vertex_dis);
	            	  res.Close_Face_num[2] = i;
	              }

	           }/* end of checking COM_A */

	         }/* End if we past a face */

             /* early exit We must be past all faces for there to be contact */
             if( !is_inside_plane )
             {
            	return res;
             }

	     } /* End of checking all faces */

	  res.is_point_inside = true;
	  return res;
}



/*---------------------------------------------------------------------------*/
/*   (4) Returns the Closest 3 faces to a vertex inside a Dpolyhedra
 *               bool is_point inside                                        */
/*---------------------------------------------------------------------------*/
__device__ PointInPoly Collision_Detection_Vertex_InDObject_3Faces(
		                                                 float3  Pos_P,
		                                                 float3  P_vertex,
                                                         uint    num_faces,
                                                         Surface  *Faces)
{

      bool   is_inside_plane;
	  float  vertex_dis ;

	  PointInPoly res;

      res.is_point_inside = false;

	  res.Close_Face_num[0] = -1;
	  res.Close_Face_num[1] = -1;
	  res.Close_Face_num[2] = -1;

	  res.distance[0] = 10.0f;
	  res.distance[1] = 10.0f;
	  res.distance[2] = 10.0f;


      /* Loop over Planes of Dynamic Objects */
	  for ( uint i=0; i< num_faces; i++ )
	  {
         is_inside_plane = false;

         /* _|_ distance between vertex and face */
         vertex_dis = dot( Faces[i].normal,( P_vertex - Faces[i].centroid ));



         /* if we past the plane report collision */
	     if( vertex_dis <= 0.000010f )
	     {
	        is_inside_plane     = true; /* point inside*/

	        /* We only want faces that the COM of the particle is on the correct side of */
	        if( dot( Faces[i].normal,( Pos_P - Faces[i].centroid ) ) > 0.0f )
	        {

	            if( fabs( vertex_dis) < res.distance[0] )
	            {
	               /* update the other 2 distances */
	               if( res.Close_Face_num[0] > -1 )
	               {
		              if( res.Close_Face_num[1] >-1 )
	                  {
		            	 res.distance  [2] = res.distance  [1];
		            	 res.Close_Face_num[2] = res.Close_Face_num[1];
	                  }
	            	  res.distance  [1] = res.distance  [0];
	            	  res.Close_Face_num[1] = res.Close_Face_num[0];
	               }

	                res.distance  [0]   = fabs(vertex_dis);
	            	res.Close_Face_num[0] = i;

	              }
	              else if( fabs(vertex_dis) < res.distance[1] )
	              {
	            	  /* update 3rd close face */
	            	  if( res.Close_Face_num[1] > -1 )
                      {
	            	     res.distance  [2] = res.distance  [1];
	            	 	 res.Close_Face_num[2] = res.Close_Face_num[1];
                      }

	            	  res.distance  [1] = fabs(vertex_dis);
	            	  res.Close_Face_num[1] = i;
	              }
	              else if( fabs(vertex_dis) < res.distance[2] )
	              {
	            	  res.distance  [2] = fabs(vertex_dis);
	            	  res.Close_Face_num[2] = i;
	              }

	           }/* end of checking COM_A */

	         }/* End if we past a face */

             /* early exit We must be past all faces for there to be contact */
             if( !is_inside_plane )
             {
            	return res;

             }

	     } /* End of checking all faces */

	  res.is_point_inside = true;
	  return res;
}







              /* START Object Collision detection */

/*---------------------------------------------------------------------------*/
/*     (4) Checks if Polyhedra is colliding with a surface (area)            */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Polyhedra_Surface( uint     index,
		                                                   uint     P_type,
		                                                   float3   Pos_P,
		                                                   float3   Vel_P,
		                                                   POBJ_ROT POBJ_RT,
		                                                   Surface  surface,
		                                                   float3  *surface_vertex  )
{

	CollData Result;

	Result.is_collision   = false;


	 /* 1) Apply Equation 1 to check position of COM */
	 if( dot( surface.normal, ( Pos_P - surface.centroid ) ) > 0.000000f )
	 {
         							   ;
          /* 3) Check distance between bounding sphere */
          if(  dot( surface.normal,( Pos_P - surface.centroid)) <= ParticleObject[P_type].radius )
          {
               /* 1. Check if a vertex is colliding */

        	    Result = Collision_Detection_Vertcies_Surface( ParticleObject[P_type].num_vertex,
        	    		                                       POBJ_RT.Vertex,
        	    		                                       surface_vertex, surface       );

        	    /* No moment for face face contact */
        	    if( Result.collision_class==Face_Face )
        	    {
        	    	Result.contact_point = Pos_P;
        	    }

        	    return Result;

           } /* End Bound Collision */

       }/* End which side we on  */

   return Result;
}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*   (5) Returns the Closest 3 faces to a vertex inside a polyhedra
 *               sface.dis = 10.0 if the vertex is not inside                */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Polyhedra_Polyhedra_Edges( Edge   *EdgesA,   Edge *EdgesB,
		                                                           uint   NumEdgesA, uint  NumEdgesB,
		                                                           float3 RelPos, float3 PosB, int num_facesB, Plane *FacesB   )
{

	CollData Result;

	float  dA,dB;
	float3 pointA;
	float3 pointB;


	float min_edgedis = 5.0f;

	int num_edgeColl=0;

	float3 selected_PA;
	float3 selected_PB;

	Result.is_collision = false;
	Result.selected_face=-1;

	/* Step one find intersecting Edges */

	bool found_vaild_Edges=false;

	Result.contact_point=make_float3(0.0f);

	  /* Now loop over all edges A */
	  for ( int i=0;i<NumEdgesA;i++ )
	  {
		     float DEAD = dot(EdgesA[i].dir,EdgesA[i].dir);

		     /* Now loop over all edges B */
		     for ( int j=0;j<NumEdgesB;j++ )
		     {
		        float3 dir = cross(EdgesA[i].dir,EdgesB[j].dir);




		        float mag    = dot(dir,dir);


		        /* Parallel Check */
		        if( mag > 0.00000f )
		        {
			        float3 DV    = EdgesA[i].Point - EdgesB[j].Point;
			        float DEBD   = dot(EdgesB[j].dir,EdgesB[j].dir);

			        float DEABD  = dot(EdgesA[i].dir,EdgesB[j].dir);

			        float DEADAB = dot(EdgesA[i].dir,DV);
			        float DEBDAB = dot(EdgesB[j].dir,DV);

			        float detJinv = 1.00000f/(DEABD*DEABD - DEAD*DEBD);


		        	dA = detJinv*(DEBD*DEADAB - DEABD*DEBDAB);


		        /* Valid point on target Edge */
		        if( dA >= -0.010f &&  dA <= 1.0100f )
		        {
		           /* Get point on approaching object */
		          dB = detJinv*(DEABD*DEADAB - DEAD*DEBDAB);

		    	   /* Now both points are valid */
			       if( dB >= -0.0100f && dB <= 1.010f )
			       {

			    	 /* Get the points on the respective Edges */
			    	 pointA = EdgesA[i].Point + EdgesA[i].dir*dA;
			    	 pointB = EdgesB[j].Point + EdgesB[j].dir*dB;

			    	  /* Compute distance between the points */
			    	  DV = ( pointA - pointB );

			    	  float dis = dot(DV,DV);

			    	  /* If we have a non-zero distance */
			    	  if(dis > 0.00000f)
			    	  {

			    		dis  = sqrt(dis);
			    	    DV*=(1.0f/dis);




                           /* Find smallest edge */
			    	       if( dis < min_edgedis )
			    	       {
			    	    	   num_edgeColl++;
			    	    	   found_vaild_Edges    = true;
		    	               min_edgedis          = dis;

		    				   Result.contact_point = pointA;
		    				   Result.Normal = cross(EdgesA[i].dir, EdgesB[j].dir);
		    		           selected_PA =pointA;
		    		           selected_PB =pointB;
			    	       }

			    	  }

			        }/* Valid points*/

		         } /* End dA valid */

		    } /* End Loop over Edges B */
		  }


	     } /* End Search */


	  if(found_vaild_Edges)
	  {
		bool valid_Point = true;
	    Result.Normal *= 1.0f/length(Result.Normal);



	  if ( (length(PosB-selected_PB) < length(PosB-selected_PA))  )
	  {
		  valid_Point = false;

          /* TODO: find a more robust collision */
		  if( isPointinPoly(num_facesB,PosB,FacesB,selected_PA) )
		  {
			  valid_Point = true;
		  }


	  }

	  if(valid_Point)
	  {


		  Result.is_collision = true;

          /* TODO: Selecting Normal is not robust use RelPos as a FIX */
		  if( fabs( dot( RelPos, Result.Normal ) ) > 0.0f)
		  {
			   Result.Normal = (dot (RelPos, Result.Normal)/fabs( dot(RelPos, Result.Normal) ) )*Result.Normal;
			   Result.dis    = fabs(dot( Result.Normal, ( selected_PB - selected_PA ) ))+0.000010f;
		  }
		  else
		  {
			 Result.Normal        = RelPos/length(RelPos);
		     Result.contact_point = PosB;
		     Result.dis           = fabs(dot( Result.Normal, ( selected_PB - selected_PA ) ))+0.000010f;
		  }

    	  Result.collision_class = Edge_Edge;

     	  return Result;
	   }
	 }
	 else/* Must be a SP */
	 {
	      Result.collision_class = SP_PLANE;
	      Result.is_collision    = false;
	 }


 	 Result.collision_class = SP_PLANE;
 	 Result.is_collision    = false;
	 return Result;
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*       (5) Checks if there is contact between 2 sets of edges                */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_DObject_Polyhedra_Edges( uint  NumEdgesDObject, Edge *EdgesDObject,
		                                                         uint  NumEdgesB, Edge *EdgesB, float3 PosB,
		                                                         int num_facesB, Plane *FacesB, float3 RelPos   )
{

	CollData Result;

	float  dA,dB;
	float3 pointA;
	float3 pointB;


	float min_edgedis = 0.10f;

	int num_edgeColl=0;

	float3 selected_PA;
	float3 selected_PB;

	Result.is_collision  = false;
	Result.selected_face = -1;

	/* Step one find intersecting Edges */

	bool found_vaild_Edges = false;

	Result.contact_point = make_float3(0.0f);

	  /* Loop over all edges A */
	  for ( int i=0;i<NumEdgesDObject;i++ )
	  {
		     float DEAD = dot(EdgesDObject[i].dir,EdgesDObject[i].dir);

		     /* Loop over all edges B */
		     for ( int j=0; j < NumEdgesB; j++ )
		     {
		    	/* check to see not parallel */
		        float3 dir = cross( EdgesDObject[i].dir/length(EdgesDObject[i].dir), EdgesB[j].dir/length(EdgesB[j].dir) );

		        float mag    = dot(dir,dir);


		        /* 1. If not Parallel Check */
		        if( mag > 0.00000f )
		        {
			        float3 DV    = EdgesDObject[i].Point - EdgesB[j].Point;

			        float DEBD   = dot(EdgesB[j].dir,EdgesB[j].dir);

			        float DEABD  = dot(EdgesDObject[i].dir,EdgesB[j].dir);

			        float DEADAB = dot(EdgesDObject[i].dir,DV);

			        float DEBDAB = dot(EdgesB[j].dir,DV);

			        float detJinv = 1.00000f/(DEABD*DEABD - DEAD*DEBD);

		        	dA = detJinv*(DEBD*DEADAB - DEABD*DEBDAB);


		            /* 2. Valid point on target Edge */
		            if( dA >= -0.010f &&  dA <= 1.0100f )
		            {

		              /* 2.1 Get point on approaching object */
		              dB = detJinv*(DEABD*DEADAB - DEAD*DEBDAB);

		    	      /* Now both points are valid */
			          if( dB >= -0.0100f && dB <= 1.010f )
			          {

					/* 3. Compute the intersection points on the respective Edges */
					pointA = EdgesDObject[i].Point + EdgesDObject[i].dir*dA;
					pointB = EdgesB[j].Point       + EdgesB[j].dir*dB;

					/* 3.1 Compute distance between the points */
					DV = ( pointA - pointB );

					float dis = dot( DV, DV );

					/* 3.2 if we have a non-zero distance (points not on each other) */
					if( dis > 0.00000f )
					{
					   dis = sqrt(dis);
					   DV *= (1.0f/dis);

					   /* Find smallest edge */
					    if ( (length( PosB - pointB) > length( PosB - pointA) )  )
					    {


							num_edgeColl++;
						    Result.contact_point += pointA;


						   found_vaild_Edges    = true;

						   if(dis<=min_edgedis)
						   {
						   min_edgedis          = dis;



						   Result.Normal        = cross(EdgesDObject[i].dir, EdgesB[j].dir);
						   selected_PA          = pointA;
						   selected_PB          = pointB;
						   }
					   }

					 }

			        }/* Valid points*/

		         } /* End dA valid */

		    } /* End Loop over Edges B */
		  }


	     } /* End Search */


	  if(found_vaild_Edges)
	  {
		bool valid_Point = true;

	    Result.Normal *= 1.0f/length(Result.Normal);

	    /* Confirm with Nico: check that there is penetration */
	    if ( (length( PosB - selected_PB) > length( PosB - selected_PA) )  )
	    {
		   valid_Point = true;
	    }

	    /* If the intersection point is inside the particle we have contact */
	    if( valid_Point )
	    {
		  Result.is_collision = true;

          /* TODO: Selecting Normal is not robust using RelPos as a FIX */

		  /* choose direction */
		  if( fabs( dot( RelPos, Result.Normal ) ) > 0.0f )
		  {
			   Result.Normal = -1.0f*(dot (RelPos, Result.Normal)/fabs( dot(RelPos, Result.Normal) ) )*Result.Normal;

			   Result.dis    =  fabs(dot( Result.Normal, ( selected_PB - selected_PA ) ));
			   Result.contact_point/=(float)num_edgeColl;

		  }
		  else/* If we choose the wrong normal close to an edge */
		  {
			 Result.Normal        = -1.0f*RelPos/length(RelPos);
		     Result.contact_point = PosB;
		     Result.dis           = fabs(dot( Result.Normal, ( selected_PB - selected_PA ) )) + 0.000010f;
		  }

    	  Result.collision_class = Edge_Edge;
    	  Result.colltype=2;

     	  return Result;
	   }
	 }
	 else/* Must be a SP */
	 {
	      Result.collision_class = SP_PLANE;
	      Result.is_collision    = false;
	 }


	 /* If the intersection point is not inside the particle then no contact*/
 	 Result.collision_class = SP_PLANE;
 	 Result.is_collision    = false;
	 return Result;
}
/*---------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
/*       (5) Checks if there is contact between 2 sets of edges                */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Polyhedra_Polyhedra_EdgesN( uint  NumEdgesA, Edge *EdgesA, float3 PosA,
		                                                            uint  NumEdgesB, Edge *EdgesB, float3 PosB,
		                                                            float3 RelPos,int ID )
{

	CollData Result;

	float  dA,dB;
	float3 pointA;
	float3 pointB;


	float max_edgedis = 0.0f;//ParticleObject[0].radius*0.10f;

	int num_edgeColl=0;

	float3 selected_PA;
	float3 selected_PB;

	Result.is_collision  = false;
	Result.selected_face = -1;

	/* Step one find intersecting Edges */

	bool found_vaild_Edges = false;

	Result.contact_point = make_float3(0.0f);

	float3 first_normal = make_float3(0.0f);
	float avg_dis=0.0f;
	bool is_many_normal =false;

	  /* Loop over all edges A */
	  for ( int i=0; i<NumEdgesA; i++ )
	  {
		     float DEAD = dot(EdgesA[i].dir,EdgesA[i].dir);

		     /* Loop over all edges B */
		     for ( int j=0; j < NumEdgesB; j++ )
		     {
		    	/* check to see not parallel */
		        float3 PDir = cross( EdgesA[i].dir/length(EdgesA[i].dir), EdgesB[j].dir/length(EdgesB[j].dir) );

		        float mag    = dot(PDir,PDir);


		        /* 1. If not Parallel Check */
		        if( mag > 0.00000f )
		        {

			        float3 DV    = EdgesA[i].Point - EdgesB[j].Point;

			        float DEBD   = dot(EdgesB[j].dir,EdgesB[j].dir);

			        float DEABD  = dot(EdgesA[i].dir,EdgesB[j].dir);

			        float DEADAB = dot(EdgesA[i].dir,DV);

			        float DEBDAB = dot(EdgesB[j].dir,DV);

			        float detJinv = 1.00000f/(DEABD*DEABD - DEAD*DEBD);

		        	dA = detJinv*(DEBD*DEADAB - DEABD*DEBDAB);


		            /* 2. Valid point on target Edge */
		            if( dA >= -0.0100f &&  dA <= 1.0100f )
		            {

		              /* 2.1 Get point on approaching object */
		              dB = detJinv*(DEABD*DEADAB - DEAD*DEBDAB);

		    	      /* Now both points are valid */
			          if( dB >= -0.0100f && dB <= 1.0100f )
			          {

					/* 3. Compute the intersection points on the respective Edges */
					pointA = EdgesA[i].Point + EdgesA[i].dir*dA;
					pointB = EdgesB[j].Point + EdgesB[j].dir*dB;

					/* Normal of the plane made by the two edges */
					PDir = cross(EdgesA[i].dir,EdgesB[j].dir);

					PDir/=length(PDir);

					/* Overlap distance between the edges */
					float dis = fabs(dot( PDir, ( pointB - pointA ) ));

					/* check that max-pen dis of 0.5% P radius is not exceeded */
					if( dis > 0.00000f  )//&& dis < ParticleObject[0].radius*0.05f)
					{
					   /* 4. Apply herustic that COM points must be on either side  */
                       float PA = dot( PDir,( PosA - (EdgesA[i].Point + EdgesB[j].Point)*0.50f) );
					   float PB = dot( PDir,( PosB - (EdgesA[i].Point + EdgesB[j].Point)*0.50f) );

					   if(  (PA/fabs(PA))*(PB/fabs(PB)) < 0.0 )
					   {
						  /* Checks if B is inside A */
						   float3 B_PB = PosB - pointB;
						   float3 B_PA = PosB - pointA;

						   float mag_BPB = length(B_PB);
						   float mag_BPA = length(B_PA);

//							  /* Checks if B is inside A */
							   float3 A_PB = PosA - pointA;
							   float3 A_PA = PosA - pointB;

							   float mag_APB = length(A_PB);
							   float mag_APA = length(A_PA);

//						   if (ID==0 && i==9 && j==8)
//						   {
//							   printf("CHECK Edge %d and %d  num %d  dis %f length %f %f \n",i,j,num_edgeColl, dis, mag_BPB,mag_BPA );
//						   }

						   /* 5. check that points are close to each other and there is actual penetration */
					      if( ( ( dot( B_PB/mag_BPB, B_PA/mag_BPA ) ) > 0.99f) && ( ( dot( A_PB/mag_APB, A_PA/mag_APA ) ) > 0.99f) && (mag_BPB > mag_BPA)  )
					      {

					    	  if (num_edgeColl==0)
					    	  {
					    		  first_normal = PDir;
					    	  }
					    	  else
					    	  {
					    		if (length(first_normal-PDir)>0.10f)
					    		{
					    			is_many_normal = true;
//								     if(ID==0)
//								     {
//								    	 printf("warning normal not consistent: ");
//								    	 PrintVectorND(first_normal);
//								    	 PrintVectorD(PDir);
//								     }
					    		 }
					    	  }


						     Result.contact_point += pointA;

						     avg_dis+=dis;
						     num_edgeColl++;

//						     if(ID==0)
//						     {
//						      printf("Edge %d and %d  num %d  dis %f length %f %f ",i,j,num_edgeColl, dis, mag_BPB,mag_BPA );
//						      //PrintVectorND(pointA);
//						      //PrintVectorND(pointB);
//						      PrintVectorD(PDir);
//						     }

						     found_vaild_Edges    = true;

						     if( dis > max_edgedis )
						     {
						       max_edgedis          = dis;
						       Result.Normal        = PDir;
						       selected_PA          = pointA;
						       selected_PB          = pointB;
						     }
					      }
					   }

					} /* End dis herustic */

			        }/* Valid points*/

		         } /* End dA valid */

		    } /* End Loop over Edges B */
		  }


	     } /* End Search */


	  if(found_vaild_Edges)
	  {
		bool valid_Point = true;

		  Result.is_collision = true;

	    /* If the intersection point is inside the particle we have contact */
	    if( valid_Point && !is_many_normal )
	    {
		    Result.Normal *= 1.0f/length(Result.Normal);


          /* TODO: Selecting Normal is not robust using RelPos as a FIX */

//		  if (ID==0)
//		  {
//		   printf(" SEdge %d and %d Num Contact Points %d \n",i,j,num_edgeColl);
//		  }

		  /* choose direction */
		  if( fabs( dot( RelPos, Result.Normal ) ) > 0.0f )
		  {
			   Result.Normal = 1.0f*(dot (RelPos, Result.Normal)/fabs( dot(RelPos, Result.Normal) ) )*Result.Normal;
			   Result.Normal /=length(Result.Normal);

			   Result.dis    =  fabs(dot( Result.Normal, ( selected_PB - selected_PA ) ));
			   Result.contact_point/=(float)num_edgeColl;

		  }
		  else/* If we choose the wrong normal close to an edge */
		  {
             //printf("using Rel pos Calc Normal: %f  %f  %f \n",Result.Normal.x,Result.Normal.y,Result.Normal.z);

			 Result.Normal        = 1.0f*RelPos/length(RelPos);
		     Result.contact_point = PosA;
		     Result.dis           = fabs(dot( Result.Normal, ( selected_PB - selected_PA ) )) + 0.000010f;
		  }

    	  Result.collision_class = Edge_Edge;
    	  Result.colltype=2;

     	  return Result;
	   }
	   else /* Use a default */
	   {
//		   if (ID==0)
//		   {
//		     printf("default edge\n");
//		   }
			 Result.Normal        = 1.0f*RelPos/length(RelPos);
		     Result.contact_point = PosA;
		     Result.dis           =avg_dis/float(num_edgeColl);//,ParticleObject[0].radius*0.010f);
	    	 Result.collision_class = Edge_Edge;
	    	 Result.colltype=2;
	     	 return Result;
	   }


	 }
	 else/* Must be a SP */
	 {
	      Result.collision_class = SP_PLANE;
	      Result.is_collision    = false;
	 }


	 /* If the intersection point is not inside the particle then no contact*/
 	 Result.collision_class = SP_PLANE;
 	 Result.is_collision    = false;
	 return Result;
}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*   (6) Checks if a vertex of A is in a face of B      */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Polyhedara_Polyhedra_VertexFaces( float3  Pos_A,
		                                                             uint    num_vertex_A,
		                                                             float3 *VertexList_A,
		                                                             float3  Pos_B,
		                                                             uint    num_faces_B,
		                                                             Plane  *FaceList_B ,
		                                                             uint    num_vertex_B,
		                                                             float3 *VertexList_B,
	                                                                 POBJ    PObjectB,

	                                                                 int      P_typeB,
	                                                                 int indexA,
	                                                                 int indexB,
	                                                                 int *contact_face=0 )
{
     CollData Result; /* Function returned */

/*---------------------------------------------------------------------------*/
/*                           TYPE 1: VERTEX FACE                             */
/*---------------------------------------------------------------------------*/

     int   num_pen_vertex = 0;

     int         valid_vertex    [8];
     PointInPoly selected_vertex [8];
     PointInPoly temp;

	 Result.is_collision = false;
     num_pen_vertex=0;

     float max_pen = 0.050*ParticleObject[P_typeB].radius;

	 /* Loop over Vertexes of Particle A and Faces particle B */
	 for ( uint i=0; i<num_vertex_A; i++ )
	 {

	  		temp = Collision_Detection_Vertex_InPoly_3Faces( Pos_A,
                                                     VertexList_A[i],
	  				                                 num_faces_B,
	  				                                 FaceList_B,
	  				                                 Pos_B  );

	  		 if( temp.is_point_inside )
	  	     {

	  			 selected_vertex [num_pen_vertex] = temp;
	  	    	 valid_vertex    [num_pen_vertex] = i;
	  	    	 num_pen_vertex++;
	  	     }
	  	  } /* End of checking all planes of A */


	  	  if( num_pen_vertex > 0 )
	  	  {
	  		Result.is_collision = true;
	  		Result.colltype     = 1;

		  	/* Single vertex face contact */
	  		if( num_pen_vertex==1 )
	  		{
	  		    Result.collision_class = Vertex_Face;

	           /* By default we use the face with the smallest distance */
		        Result.dis           = selected_vertex[0].distance[0];
		    	Result.Normal        = FaceList_B[selected_vertex[0].Close_Face_num[0]].normal;
		    	Result.contact_point = VertexList_A[valid_vertex[0]];


		    	/* only check if there is another face that is valid */
		    	if( selected_vertex[0].Close_Face_num[1]==-1 )
		    	{
		    	  if(SimParms.use_hist)
		    	  {
		    	   contact_face[0] = selected_vertex[0].Close_Face_num[0];
		    	  }
		    	  return Result;
		    	}
		    	else
		    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
                  float3 Ray = VertexList_A[valid_vertex[0]] - Pos_A;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[0]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[0]].centroid - Pos_A) ,
						    FaceList_B[selected_vertex[0].Close_Face_num[0]].normal)/dir;


				     if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[0]].num_vertex,
									  VertexList_B,
									  PObjectB.face[selected_vertex[0].Close_Face_num[0]].vertex_Order,
									  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[0]].area,0.10f,Pos_B ))
				     {
				       if(SimParms.use_hist)
				       {
				    	 contact_face[0] = selected_vertex[0].Close_Face_num[0];
				       }
					   return Result;
				     }
				    }

		    	}



		    	/* check if 2nd face is valid */
		    	if( selected_vertex[0].Close_Face_num[1] > -1  )
		    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
                  float3 Ray = VertexList_A[valid_vertex[0]] - Pos_A;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[1]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[1]].centroid - Pos_A) ,
						    FaceList_B[selected_vertex[0].Close_Face_num[1]].normal)/dir;


				     if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[1]].num_vertex,
									  VertexList_B,
									  PObjectB.face[selected_vertex[0].Close_Face_num[1]].vertex_Order,
									  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[1]].area,0.10f,Pos_B ))
				     {
				       Result.Normal = FaceList_B[selected_vertex[0].Close_Face_num[1]].normal;
				       Result.dis           = selected_vertex[0].distance[1];

				    	  if(SimParms.use_hist)
				    	  {
				       contact_face[0] = selected_vertex[0].Close_Face_num[1];
				    	  }
					   return Result;
				     }
				   }
		    	}


		    	/* check if 3rd face is valid */
		    	if( selected_vertex[0].Close_Face_num[2]>-1 )
		    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
                  float3 Ray = VertexList_A[valid_vertex[0]] - Pos_A;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[2]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[2]].centroid - Pos_A) ,
						    FaceList_B[selected_vertex[0].Close_Face_num[2]].normal)/dir;


				     if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[2]].num_vertex,
									  VertexList_B,
									  PObjectB.face[selected_vertex[0].Close_Face_num[2]].vertex_Order,
									  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[2]].area,0.10f,Pos_B ) )
				     {
					       Result.Normal = FaceList_B[selected_vertex[0].Close_Face_num[2]].normal;
					       Result.dis    = selected_vertex[0].distance[2];

					    	  if(SimParms.use_hist)
					    	  {
					       contact_face[0] = selected_vertex[0].Close_Face_num[2];
					    	  }
					   return Result;
				     }
				   }
		    	}

				 printf(" Warning VF Logic Error \n");
				 return Result;


	  		}


	  		/* EDGE FACE */
	  		if (num_pen_vertex==2)
	  		{
	  		  Result.is_collision    = true;
	  		  Result.colltype        = 1;
	  		  Result.collision_class = Edge_Face;

	  		  Result.contact_point   = (VertexList_A[valid_vertex[0]]
	  		                           + VertexList_A[valid_vertex[1]])*0.50;

	  		  Result.dis = fabs(dot(FaceList_B[selected_vertex[0].Close_Face_num[0]].normal,
	                     (Result.contact_point-FaceList_B[selected_vertex[0].Close_Face_num[0]].centroid)));

	  		  Result.Normal = FaceList_B[selected_vertex[0].Close_Face_num[0]].normal;



	    	/* only check if there is another face that is valid */
	    	if( selected_vertex[0].Close_Face_num[1]==-1 && selected_vertex[1].Close_Face_num[1]==-1 )
	    	{
		      if(SimParms.use_hist)
		      {
	    	    contact_face[0] = selected_vertex[0].Close_Face_num[0];
		      }
	    	  return Result;
	    	}
	    	else
	    	{
	    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
              float3 Ray = Result.contact_point - Pos_A;
			       Ray/=length(Ray);

			  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[0]].normal );

			   /* check that there is a common component at-least */
			   if ( fabs(dir) > 0 )
			   {
			    /* compute where the ray intersects the plane of the face  */
			    float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[0]].centroid - Pos_A) ,
					    FaceList_B[selected_vertex[0].Close_Face_num[0]].normal)/dir;


			     if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[0]].num_vertex,
								  VertexList_B,
								  PObjectB.face[selected_vertex[0].Close_Face_num[0]].vertex_Order,
								  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[0]].area,0.10f,Pos_B ) )
			     {
			    	 /* first face is correct */
				        Result.dis           =  fabs(dot(FaceList_B[selected_vertex[0].Close_Face_num[0]].normal,
				        		                     (Result.contact_point-FaceList_B[selected_vertex[0].Close_Face_num[0]].centroid)));
				    	Result.Normal        = FaceList_B[selected_vertex[0].Close_Face_num[0]].normal;
				   return Result;
			     }

			    }

	    	}


	    	/* check if 2nd face is valid */
	    	if( selected_vertex[0].Close_Face_num[1]>-1  )
	    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
	              float3 Ray = Result.contact_point - Pos_A;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[1]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[1]].centroid - Pos_A) ,
						    FaceList_B[selected_vertex[0].Close_Face_num[1]].normal)/dir;


				     if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[1]].num_vertex,
									  VertexList_B,
									  PObjectB.face[selected_vertex[0].Close_Face_num[1]].vertex_Order,
									  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[1]].area,0.10f,Pos_B ) )
				     {
				    	 /* first face is correct */
					        Result.dis           =  fabs(dot(FaceList_B[selected_vertex[0].Close_Face_num[1]].normal,
					        		                     (Result.contact_point-FaceList_B[selected_vertex[0].Close_Face_num[1]].centroid)));
					    	Result.Normal        = FaceList_B[selected_vertex[0].Close_Face_num[1]].normal;

					    	  if(SimParms.use_hist)
					    	  {
					    	contact_face[0] = selected_vertex[0].Close_Face_num[1];
					    	  }
					   return Result;
				     }

				    }
	    	}


	    	/* check if 3rd face is valid */
	    	if(selected_vertex[0].Close_Face_num[2]>-1  )
	    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
	              float3 Ray = Result.contact_point - Pos_A;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[2]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[2]].centroid - Pos_A) ,
						    FaceList_B[selected_vertex[0].Close_Face_num[2]].normal)/dir;


				     if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[2]].num_vertex,
									  VertexList_B,
									  PObjectB.face[selected_vertex[0].Close_Face_num[2]].vertex_Order,
									  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[2]].area,0.10f,Pos_B ) )
				     {
				    	 /* first face is correct */
					        Result.dis           =  fabs(dot(FaceList_B[selected_vertex[0].Close_Face_num[2]].normal,
					        		                     (Result.contact_point-FaceList_B[selected_vertex[0].Close_Face_num[2]].centroid)));
					    	Result.Normal        = FaceList_B[selected_vertex[0].Close_Face_num[2]].normal;

					    	  if(SimParms.use_hist)
					    	  {
					    	contact_face[0] = selected_vertex[0].Close_Face_num[2];
					    	  }
					    	return Result;
				     }


				    }
	    	}



	  		   printf(" Warning Edge-Face logic error \n");
	  		   return Result;

	  		}/* End Edge Face */


	  		/* FACE FACE: TODO Check that assumption that only one face will have all vertexes
	  		 *                 and if ray tracing is not cheaper */

	  		if (num_pen_vertex>2)
	  		{
		  		Result.is_collision = true;
		  		Result.colltype = 1;
		  		Result.collision_class = Face_Face;
                Result.contact_point = make_float3(0.0f);

                /* Sum all penetrated vertex */
    		  	for (int i=0; i<num_pen_vertex; i++)
		  		{
		  		   Result.contact_point += VertexList_A[valid_vertex[i]];
		  		}
		  		Result.contact_point/=(float)num_pen_vertex;

				/* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
				float3 Ray = Result.contact_point - Pos_A;
					   Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[0]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
					/* compute where the ray intersects the plane of the face  */
					float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[0]].centroid - Pos_A) ,
							FaceList_B[selected_vertex[0].Close_Face_num[0]].normal)/dir;


					 if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[0]].num_vertex,
									  VertexList_B,
									  PObjectB.face[selected_vertex[0].Close_Face_num[0]].vertex_Order,
									  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[0]].area,0.10f,Pos_B ) )
					 {
						 /* first face is correct */
							Result.dis           =  fabs(dot(FaceList_B[selected_vertex[0].Close_Face_num[0]].normal,
														 (Result.contact_point-FaceList_B[selected_vertex[0].Close_Face_num[0]].centroid)));
							Result.Normal        = FaceList_B[selected_vertex[0].Close_Face_num[0]].normal;

					    	  if(SimParms.use_hist)
					    	  {
							contact_face[0] = selected_vertex[0].Close_Face_num[0];
					    	  }
					        return Result;
					 }

					}


				/* check if 2nd face is valid */
				if(selected_vertex[0].Close_Face_num[1]>-1  )
				{
					  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
					  float3 Ray = Result.contact_point - Pos_A;
						   Ray/=length(Ray);

					  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[1]].normal );

					   /* check that there is a common component at-least */
					   if ( fabs(dir) > 0 )
					   {
						/* compute where the ray intersects the plane of the face  */
						float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[1]].centroid - Pos_A) ,
								FaceList_B[selected_vertex[0].Close_Face_num[1]].normal)/dir;


						 if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[1]].num_vertex,
										  VertexList_B,
										  PObjectB.face[selected_vertex[0].Close_Face_num[1]].vertex_Order,
										  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[1]].area,0.010f,Pos_B ))
						 {
							 /* second face is correct */
								Result.dis           =  fabs(dot(FaceList_B[selected_vertex[0].Close_Face_num[1]].normal,
															 (Result.contact_point-FaceList_B[selected_vertex[0].Close_Face_num[1]].centroid)));
								Result.Normal        = FaceList_B[selected_vertex[0].Close_Face_num[1]].normal;
						  		if (Result.dis>max_pen)
						  		{
						  			printf("ERROR Second face to large \n");
						  		}
						    	  if(SimParms.use_hist)
						    	  {
						  		contact_face[0] = selected_vertex[0].Close_Face_num[1];
						    	  }
						        return Result;
						 }

						}
				}


				/* check if 3rd face is valid */
				if(selected_vertex[0].Close_Face_num[2]>-1  )
				{
					  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
					  float3 Ray = Result.contact_point - Pos_A;
						   Ray/=length(Ray);

					  float dir = dot( Ray, FaceList_B[selected_vertex[0].Close_Face_num[2]].normal );

					   /* check that there is a common component at-least */
					   if ( fabs(dir) > 0 )
					   {
						/* compute where the ray intersects the plane of the face  */
						float d = dot(  (FaceList_B[selected_vertex[0].Close_Face_num[2]].centroid - Pos_A) ,
								FaceList_B[selected_vertex[0].Close_Face_num[2]].normal)/dir;


						 if( is_PointOnSurface( PObjectB.face[selected_vertex[0].Close_Face_num[2]].num_vertex,
										  VertexList_B,
										  PObjectB.face[selected_vertex[0].Close_Face_num[2]].vertex_Order,
										  Pos_A + d*Ray,PObjectB.face[selected_vertex[0].Close_Face_num[2]].area,0.10f,Pos_B ))
						 {
							 /* first face is correct */
								Result.dis           =  fabs(dot(FaceList_B[selected_vertex[0].Close_Face_num[2]].normal,
															 (Result.contact_point-FaceList_B[selected_vertex[0].Close_Face_num[2]].centroid)));
								Result.Normal        = FaceList_B[selected_vertex[0].Close_Face_num[2]].normal;

						    	if(SimParms.use_hist)
						    	{
								  contact_face[0] = selected_vertex[0].Close_Face_num[2];
						    	}

						  		if (Result.dis>max_pen)
						  		{
						  			printf("ERROR Third face to large \n");
						  		}

						   return Result;
						 }


						}
				}

			    printf(" Warning Face Logic Error \n");
		  		return Result;

	  		}/* End Edge Face */


	  	  }


 return Result;

}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*   (6) Checks if a vertex of A is in a face of B      */
/*---------------------------------------------------------------------------*/
__device__ CollData Collision_Detection_Polyhedara_VolumeObject_VertexFace( float3  Pos_P,
		                                                               uint    num_vertex_P,
		                                                               float3 *VertexList_P,

		                                                               uint    num_faces_DObject,
		                                                               Surface *FaceList_DObject ,
		                                                               uint    num_vertex_DObject,
		                                                               float3 *VertexList_DObject )
{
     CollData Result; /* Function returned */

/*---------------------------------------------------------------------------*/
/*                           TYPE 1: VERTEX FACE                             */
/*---------------------------------------------------------------------------*/

     int   num_pen_vertex = 0;

     int         valid_vertex    [8];
     PointInPoly selected_vertex [8];
     PointInPoly temp;

	 Result.is_collision = false;
	 Result.collision_class = UNDEF;

     num_pen_vertex=0;

     //float max_pen = 0.50*ParticleObject[P_typeB].radius;

	 /* Loop over Vertexes of Particle A and Faces particle B */
	 for ( uint i=0; i<num_vertex_P; i++ )
	 {

	  		temp = Collision_Detection_Vertex_InDObject_3Faces( Pos_P,
                                                                VertexList_P[i],
	  				                                            num_faces_DObject,
	  				                                            FaceList_DObject   );

	  		 if( temp.is_point_inside )
	  	     {

	  			 selected_vertex [num_pen_vertex] = temp;
	  	    	 valid_vertex    [num_pen_vertex] = i;
	  	    	 num_pen_vertex++;
	  	     }
	  } /* End of checking all Vertex of Particle */


	      /* If we have a single penetrated vertex */
	  	  if( num_pen_vertex > 0 )
	  	  {

	  		Result.is_collision = true;
	  		Result.colltype     = 1;

		  	  /* Single vertex face contact */
	  		if( num_pen_vertex==1 )
	  		{
	  		    Result.collision_class = Vertex_Face;

	           /* By default we use the face with the smallest distance */
		        Result.dis           = selected_vertex[0].distance[0];
		    	Result.Normal        = FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal;
		    	Result.contact_point = VertexList_P[valid_vertex[0]];


		    	/* only check if there is another face that is valid */
		    	if( selected_vertex[0].Close_Face_num[1] == -1 )
		    	{
		    	  return Result;
		    	}
		    	else
		    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
                  float3 Ray = VertexList_P[valid_vertex[0]] - Pos_P;
				         Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_DObject[selected_vertex[0].Close_Face_num[0]].centroid - Pos_P) ,
				    		FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal)/dir;


				     if( is_PointOnSurface( FaceList_DObject[selected_vertex[0].Close_Face_num[0]].num_vertex,
									  VertexList_DObject,
									  FaceList_DObject[selected_vertex[0].Close_Face_num[0]].vertex_Order,
									  Pos_P + d*Ray,FaceList_DObject[selected_vertex[0].Close_Face_num[0]].area,0.1f ))
				     {
					   return Result;
				     }
				    }

		    	}



		    	/* check if 2nd face is valid */
		    	if(selected_vertex[0].Close_Face_num[1]>-1  )
		    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
                  float3 Ray = VertexList_P[valid_vertex[0]] - Pos_P;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_DObject[selected_vertex[0].Close_Face_num[1]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_DObject[selected_vertex[0].Close_Face_num[1]].centroid - Pos_P) ,
				    		FaceList_DObject[selected_vertex[0].Close_Face_num[1]].normal)/dir;


				     if( is_PointOnSurface( FaceList_DObject[selected_vertex[0].Close_Face_num[1]].num_vertex,
									  VertexList_DObject,
									  FaceList_DObject[selected_vertex[0].Close_Face_num[1]].vertex_Order,
									  Pos_P + d*Ray,FaceList_DObject[selected_vertex[0].Close_Face_num[1]].area,0.10f ) )
				     {
				       Result.Normal = FaceList_DObject[selected_vertex[0].Close_Face_num[1]].normal;
				       Result.dis           = selected_vertex[0].distance[1];
					   return Result;
				     }
				   }
		    	}


		    	/* check if 3rd face is valid */
		    	if( selected_vertex[0].Close_Face_num[2]>-1 )
		    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
                  float3 Ray = VertexList_P[valid_vertex[0]] - Pos_P;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_DObject[selected_vertex[0].Close_Face_num[2]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_DObject[selected_vertex[0].Close_Face_num[2]].centroid - Pos_P) ,
				    		FaceList_DObject[selected_vertex[0].Close_Face_num[2]].normal)/dir;


				     if( is_PointOnSurface( FaceList_DObject[selected_vertex[0].Close_Face_num[2]].num_vertex,
									  VertexList_DObject,
									  FaceList_DObject[selected_vertex[0].Close_Face_num[2]].vertex_Order,
									  Pos_P + d*Ray,FaceList_DObject[selected_vertex[0].Close_Face_num[2]].area,0.10f ) )
				     {
					       Result.Normal = FaceList_DObject[selected_vertex[0].Close_Face_num[2]].normal;
					       Result.dis    = selected_vertex[0].distance[2];
					   return Result;
				     }
				   }
		    	}

				 //printf(" VF Default \n");
				 return Result;


	  		}


	  		/* EDGE FACE */
	  		if (num_pen_vertex==2)
	  		{
	  		  Result.is_collision    = true;
	  		  Result.colltype        = 1;
	  		  Result.collision_class = Edge_Face;

	  		  Result.contact_point   = (VertexList_P[valid_vertex[0]]
	  		                           + VertexList_P[valid_vertex[1]])*0.50;

	  		  Result.Normal        = FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal;

	  		  Result.dis = fabs( dot( FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal,
	                            (Result.contact_point-FaceList_DObject[selected_vertex[0].Close_Face_num[0]].centroid)) );




	    	/* only check if there is another face that is valid */
	    	if( selected_vertex[0].Close_Face_num[1]==-1 && selected_vertex[1].Close_Face_num[1]==-1 )
	    	{
	    	  return Result;
	    	}
	    	else
	    	{
	    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
              float3 Ray = Result.contact_point - Pos_P;
			       Ray/=length(Ray);

			  float dir = dot( Ray, FaceList_DObject [selected_vertex[0].Close_Face_num[0]].normal );

			   /* check that there is a common component at-least */
			   if ( fabs(dir) > 0 )
			   {
			    /* compute where the ray intersects the plane of the face  */
			    float d = dot(  (FaceList_DObject[selected_vertex[0].Close_Face_num[0]].centroid - Pos_P) ,
			    		FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal)/dir;


			     if( is_PointOnSurface( FaceList_DObject[selected_vertex[0].Close_Face_num[0]].num_vertex,
								  VertexList_DObject,
								  FaceList_DObject[selected_vertex[0].Close_Face_num[0]].vertex_Order,
								  Pos_P + d*Ray,FaceList_DObject[selected_vertex[0].Close_Face_num[0]].area, 0.10f ) )
			     {
			    	 /* first face is correct */
				        Result.dis           =  fabs(dot(FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal,
				        		                     (Result.contact_point-FaceList_DObject[selected_vertex[0].Close_Face_num[0]].centroid)));
				    	Result.Normal        = FaceList_DObject[selected_vertex[0].Close_Face_num[0]].normal;
				   return Result;
			     }

			    }

	    	}


	    	/* check if 2nd face is valid */
	    	if(selected_vertex[0].Close_Face_num[1]>-1  )
	    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
	              float3 Ray = Result.contact_point - Pos_P;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_DObject[selected_vertex[0].Close_Face_num[1]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_DObject[selected_vertex[0].Close_Face_num[1]].centroid - Pos_P) ,
				    		FaceList_DObject[selected_vertex[0].Close_Face_num[1]].normal)/dir;


				     if( is_PointOnSurface( FaceList_DObject[selected_vertex[0].Close_Face_num[1]].num_vertex,
									  VertexList_DObject,
									  FaceList_DObject[selected_vertex[0].Close_Face_num[1]].vertex_Order,
									  Pos_P + d*Ray,FaceList_DObject[selected_vertex[0].Close_Face_num[1]].area,0.10f ) )
				     {
				    	 /* first face is correct */
					        Result.dis           =  fabs(dot(FaceList_DObject[selected_vertex[0].Close_Face_num[1]].normal,
					        		                     (Result.contact_point-FaceList_DObject[selected_vertex[0].Close_Face_num[1]].centroid)));
					    	Result.Normal        = FaceList_DObject[selected_vertex[0].Close_Face_num[1]].normal;
					   return Result;
				     }

				    }
	    	}


	    	/* check if 3rd face is valid */
	    	if(selected_vertex[0].Close_Face_num[2]>-1  )
	    	{
		    	  /* Ray Points toward B in Direction of the contact vertex so Pos_A is the start */
	              float3 Ray = Result.contact_point - Pos_P;
				       Ray/=length(Ray);

				  float dir = dot( Ray, FaceList_DObject[selected_vertex[0].Close_Face_num[2]].normal );

				   /* check that there is a common component at-least */
				   if ( fabs(dir) > 0 )
				   {
				    /* compute where the ray intersects the plane of the face  */
				    float d = dot(  (FaceList_DObject[selected_vertex[0].Close_Face_num[2]].centroid - Pos_P) ,
				    		FaceList_DObject[selected_vertex[0].Close_Face_num[2]].normal)/dir;


				     if( is_PointOnSurface( FaceList_DObject[selected_vertex[0].Close_Face_num[2]].num_vertex,
									  VertexList_DObject,
									  FaceList_DObject[selected_vertex[0].Close_Face_num[2]].vertex_Order,
									  Pos_P + d*Ray, FaceList_DObject[selected_vertex[0].Close_Face_num[2]].area, 0.10f ) )
				     {
				    	 /* first face is correct */
					        Result.dis           =  fabs(dot(FaceList_DObject[selected_vertex[0].Close_Face_num[2]].normal,
					        		                     (Result.contact_point-FaceList_DObject[selected_vertex[0].Close_Face_num[2]].centroid)));
					    	Result.Normal        = FaceList_DObject[selected_vertex[0].Close_Face_num[2]].normal;
					   return Result;
				     }


				    }
	    	}
	  		   return Result;

	  		}/* End Edge Face */


	  		/* FACE FACE: TODO Check that assumption that only one face will have all vertexes
	  		 *                 and if ray tracing is not cheaper */
	  		if (num_pen_vertex>2)
	  		{
		  		  Result.is_collision = true;
		  		  Result.colltype = 1;
		  		  Result.collision_class = Face_Face;

    		  	  Result.contact_point = make_float3(0.0f);

    		  	  for (int i=0; i<num_pen_vertex; i++)
		  		  {
		  		   Result.contact_point += VertexList_P[valid_vertex[i]];
		  		  }
		  		  Result.contact_point/=(float)num_pen_vertex;


				   uint   common_face    [32];
				   float  common_face_dis[32];
				   common_face[0] = 0;
				   common_face[1] = 0;
				   common_face[2] = 0;

				   common_face_dis[0] = 10.0;
				   common_face_dis[1] = 10.0;
				   common_face_dis[2] = 10.0;

				  /* Loop over selected vertex */
				  for ( int  i=0; i<num_pen_vertex; i++)
				  {
					/* Loop over selected faces */
					for ( int  j=0; j<3; j++)
					{
					  /* only valid faces */
					  if( selected_vertex[i].Close_Face_num[j] > -1 )
					  {
						 /* increment face count */
						 common_face[selected_vertex[i].Close_Face_num[j]]++;

						 /* store maximum distance for that face */
						 if( selected_vertex[i].distance[j] < common_face_dis[selected_vertex[i].Close_Face_num[j]])
						 {
							common_face_dis[selected_vertex[i].Close_Face_num[j]] = selected_vertex[i].distance[j];
						 }
					   }
					}
				  }

				  /* only one face will have all vertexes passing thru */
				   for ( int i=0; i<num_faces_DObject; i++ )
				   {
					  if(common_face[i]==num_pen_vertex)
					  {
                        Result.dis = common_face_dis[i];
                        Result.Normal = FaceList_DObject[i].normal;
						return Result;
					  }
				   }


			    //printf(" ERROR NO Face found \n");
		  		 return Result;



	  		}/* End Edge Face */


	  	  }


 return Result;

}
/*------------------------------------*/



/*___________________________________________________________________________*/
/*   (4) Determines the actual vertex in contact with a cylinder surface     */
/*___________________________________________________________________________*/
__device__ CollData Collision_Detection_Polyhedra_Cylinder( uint           index,
		                                                    uint           P_type,
		                                                    float3         Pos_P,
		                                                    float3         Vel_P,
		                                                    Macro_Cylinder Cyl,
		                                                    POBJ_ROT       POBJ_RT    )
{

	CollData Result;
	Result.is_collision = false;

    float3 AXIS = Cyl.AXIS;
	float3 Pvec = Pos_P*AXIS - Cyl.center_bot_cap;/* Relative to cent*/


	float Pvec_dis  = length(Pvec) + ParticleObject[P_type].radius;

    /* Check if bounding sphere intersects cylinder */
    if (  Pvec_dis > Cyl.radius   )
    {

    	float3 v_point;       /* vertex of particle */
    	float  vertex_dis;    /* distance to coll   */

    	float  dis = 0.0;


    	int num_vertexCount = 0;
    	float3 s_point      = make_float3(0.0f);

        /* Find colliding Vertex */
        for( uint i=0; i<ParticleObject[P_type].num_vertex; i++ )
        {

    	    /* Vertex position relative to cent of cylinder */
    	    v_point = ( POBJ_RT.Vertex[i] )*AXIS - Cyl.center_bot_cap  ;

    	    float rc = length(v_point);

            /* Check if vertex point is intersecting */
           if( rc > Cyl.radius )
    	   {

         	  vertex_dis =  rc - Cyl.radius ;

        	     /* Increment contact points */
        	       num_vertexCount++;

        	       /* Average contact points */
        	       s_point  += POBJ_RT.Vertex[i];

        	            dis  += vertex_dis;

        	       /* Exit Face Contact */
        	       if(num_vertexCount>2)
        	       {
        	    	  dis*=1.0/((float)num_vertexCount);
        	         Result.is_collision   = true;
        	         Result.collision_class = Face_Face;

        	         Result.Normal  = -Pvec;//1.0f*( Cyl.normal_bot_cap - Result.body_point*Cyl.AXIS );
        	         Result.Normal *= (1.0f/length(Result.Normal));
        	         Result.dis            = min(0.00100f,fabs(dis));
                     /* no moment for face contact */
        	         Result.contact_point = Pos_P;

        	         return Result;

        	       }


    	   }

        }

        /* No Collision particle can be moved */
        if( num_vertexCount==0 )
        {
          Result.is_collision = false;
          return Result;
        }

	       if(num_vertexCount==2)
	       {

	    	   Result.is_collision   = true;
	    	   dis*=1.0/((float)num_vertexCount);
	         Result.is_collision   = true;
	         Result.collision_class = Edge_Face;
	         Result.contact_point     = s_point/((float)num_vertexCount);

	         Result.Normal  = -Pvec;//-1.0f*( Cyl.normal_bot_cap - Result.body_point*Cyl.AXIS );
	         Result.Normal *= (1.0f/length(Result.Normal));
	         Result.dis            = fabs(dis);
	         /* Contact point world coordinates */



	         return Result;

	       }

	       dis*=1.0/((float)num_vertexCount);

        /* get contact point */
        Result.contact_point     = s_point;
        /* get the normal to the surface */
        Result.Normal  = -Pvec;//-1.0f*( Cyl.normal_bot_cap - Result.body_point*Cyl.AXIS );
        Result.Normal *= (1.0f/length(Result.Normal));
        Result.dis     = dis;
        Result.is_collision   = true;
        Result.collision_class = Vertex_Face;


        return Result;


    }



   return Result;
}
/*___________________________________________________________________________*/







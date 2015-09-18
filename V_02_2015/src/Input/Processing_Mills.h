


/*---------------------------------------------------------------------------*/
/*        Rotates a Dynamic Object Incrementally(Variable Speed)             */
/*---------------------------------------------------------------------------*/
void Rotate_VolObject_Inc(float3 about)
{


  for( int index=0;index<m_num_KDynamicObjects;index++ )
  {

	if(m_KDynamicObject[index].is_attached_DWorld )
	{
	 float3 sum = make_float3(0.0f,0.0f,0.0f);

     /* Rotate Vertex */
     for(int k=0;k<m_KDynamicObject[index].num_vertex;k++)
     {
    	 float3 point = m_KDynamicObject[index].vertex[k] - about ;

    	 m_KDynamicObject[index].vertex[k].x = cos(m_Mill_SimData.RadPerStep)*point.x + sin(m_Mill_SimData.RadPerStep)*point.y;
    	 m_KDynamicObject[index].vertex[k].y = -sin(m_Mill_SimData.RadPerStep)*point.x + cos(m_Mill_SimData.RadPerStep)*point.y;
    	 m_KDynamicObject[index].vertex[k] = m_KDynamicObject[index].vertex[k] + about;

     }

    // printf("%f\n",m_KDynamicObject[index].vertex[0].x);
     /* Update Surfaces */
     for(int j=0;j<m_KDynamicObject[index].num_faces;j++)
     {
    	 sum = make_float3(0.0f,0.0f,0.0f);

         for(int f=0;f<m_KDynamicObject[index].faces[j].num_vertex;f++)
         {
	       sum = sum + m_KDynamicObject[index].vertex[ m_KDynamicObject[index].faces[j].vertex_Order[f] ];
         }

         m_KDynamicObject[index].faces[j].centroid = (1.0f/m_KDynamicObject[index].faces[j].num_vertex)*sum;

         float3  V1 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[1]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[0]];

         float3  V2 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[2]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[1]];


         float3  V3 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[3]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[2]];


         float3  V4 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[0]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[3]];


         float3 n         = cross(V2,V1);

         m_KDynamicObject[index].faces[j].normal = n/length(n);

         m_KDynamicObject[index].faces[j].area= 0.500000f*length(n) + 0.500000f*length(cross(V3,V4));

     }


     /* Rotate COM */
     float3 point = m_KDynamicObject[index].COM - about;
     m_KDynamicObject[index].COM.x = cos(m_Mill_SimData.RadPerStep)*point.x + sin(m_Mill_SimData.RadPerStep)*point.y;
     m_KDynamicObject[index].COM.y = -sin(m_Mill_SimData.RadPerStep)*point.x + cos(m_Mill_SimData.RadPerStep)*point.y;

     m_KDynamicObject[index].COM   = m_KDynamicObject[index].COM + about;

     }
  }

  if(m_OpenGL.render)
  {
     Draw_DynamicGeometry();
  }

     /* Update the Device */
     m_KDevice->IDevice_UpdateVolObjectPositions(m_KDynamicObject);
}


/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*        Rotates a Dynamic Object Incrementally(Variable Speed)             */
/*---------------------------------------------------------------------------*/
void Rotate_VolObject_angle(float3 about)
{


  for( int index=0;index<m_num_KDynamicObjects;index++ )
  {

	if(m_KDynamicObject[index].is_attached_DWorld )
    {

	 float3 sum = make_float3(0.0f,0.0f,0.0f);


     /* Rotate Vertex */
     for(int k=0;k<m_KDynamicObject[index].num_vertex;k++)
     {
    	 float3 point = m_KDynamicObject_Initial[index].vertex[k] - about ;

    	 m_KDynamicObject[index].vertex[k].x = cos(Rotation_Angle)*point.x + sin(Rotation_Angle)*point.y;
    	 m_KDynamicObject[index].vertex[k].y = -sin(Rotation_Angle)*point.x + cos(Rotation_Angle)*point.y;
    	 m_KDynamicObject[index].vertex[k] += about;

     }


     //printf("%f\n",m_KDynamicObject[index].vertex[0].x);

     /* Update Surfaces */
     for(int j=0;j<m_KDynamicObject[index].num_faces;j++)
     {
    	 sum = make_float3(0.0f,0.0f,0.0f);

         for(int f=0;f<m_KDynamicObject[index].faces[j].num_vertex;f++)
         {
	       sum = sum + m_KDynamicObject[index].vertex[ m_KDynamicObject[index].faces[j].vertex_Order[f] ];
         }

         m_KDynamicObject[index].faces[j].centroid = (1.0f/m_KDynamicObject[index].faces[j].num_vertex)*sum;

         float3  V1 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[1]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[0]];

         float3  V2 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[2]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[1]];


         float3  V3 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[3]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[2]];


         float3  V4 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[0]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[3]];


         float3 n         = cross(V2,V1);

         m_KDynamicObject[index].faces[j].normal = n/length(n);
         m_KDynamicObject[index].faces[j].area= 0.500000f*length(n) + 0.500000f*length(cross(V3,V4));

     }


     /* Rotate COM */
     float3 point = m_KDynamicObject[index].COM - about;
     m_KDynamicObject[index].COM.x = cos(Rotation_Angle)*point.x + sin(Rotation_Angle)*point.y;
     m_KDynamicObject[index].COM.y = -sin(Rotation_Angle)*point.x + cos(Rotation_Angle)*point.y;

     m_KDynamicObject[index].COM   = m_KDynamicObject[index].COM + about;
    }
  }

  if(m_OpenGL.render)
  {
     Draw_DynamicGeometry();
  }

     /* Update the Device */
     m_KDevice->IDevice_UpdateVolObjectPositions(m_KDynamicObject);
}



/*---------------------------------------------------------------------------*/
/*     Rotates a World Object from Start Position(Variable Speed)            */
/*---------------------------------------------------------------------------*/
void Rotate_WObject_angle(float3 about)
{

  for( int index=0; index< m_num_KWorldObjects; index++ )
  {

	if( m_KWorldObject[index].is_rotating )
	{
      float3 sum = make_float3(0.0f,0.0f,0.0f);

      /* Rotate all WObject Vertex about the same point  */
      for(int k=0;k<m_KWorldObject[index].num_vertex;k++)
      {
    	 float3 point = m_KWorldObject_Initial[index].vertex[k] - about ;/* Get vertex relative to point */

    	 m_KWorldObject[index].vertex[k].x =  cos(Rotation_Angle)*point.x + sin(Rotation_Angle)*point.y;
    	 m_KWorldObject[index].vertex[k].y = -sin(Rotation_Angle)*point.x + cos(Rotation_Angle)*point.y;
    	 m_KWorldObject[index].vertex[k]   = m_KWorldObject[index].vertex[k] + about; /* Now Translate back */
       }

       /* Update Surfaces based on new vertex positions */
       for(int j=0;j<m_KWorldObject[index].num_surfaces;j++)
       {
    	 sum = make_float3(0.0f,0.0f,0.0f);

         for( int f=0; f<m_KWorldObject[index].surfaces[j].num_vertex; f++ )
         {
	       sum = sum + m_KWorldObject[index].vertex[ m_KWorldObject[index].surfaces[j].vertex_Order[f] ];
         }

         m_KWorldObject[index].surfaces[j].centroid = (1.0f/m_KWorldObject[index].surfaces[j].num_vertex)*sum;

         float3  V1 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[1]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[0]];

         float3  V2 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[2]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[1]];


         /* Compute new normal */
         float3 n         = cross(V2,V1);
         m_KWorldObject[index].surfaces[j].normal = n/length(n);

         if(m_KWorldObject[index].surfaces[j].num_vertex>3)
         {
         float3  V3 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[3]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[2]];


         float3  V4 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[0]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[3]];

              m_KWorldObject[index].surfaces[j].area= 0.500000f*length(n) + 0.500000f*length(cross(V3,V4));
         }
         else
         {
        	 m_KWorldObject[index].surfaces[j].area= 0.500000f*length(n);
         }





       }

	}/* end world object*/

  }


     /* Update the Device */
     m_KDevice->IDevice_UpdateWorldObjects(m_KWorldObject);
}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*             Rotates a World Object from Start Position                  */
/*---------------------------------------------------------------------------*/
void Rotate_WObject_inc(float3 about)
{


  for( int index=0; index< m_num_KWorldObjects; index++ )
  {

	if( m_KWorldObject[index].is_rotating )
	{
      float3 sum = make_float3(0.0f,0.0f,0.0f);

      /* Rotate all WObject Vertex about the same point  */
      for(int k=0;k<m_KWorldObject[index].num_vertex;k++)
      {
    	 float3 point = m_KWorldObject[index].vertex[k] - about ;/* Get vertex relative to point */

    	 m_KWorldObject[index].vertex[k].x =  cos(m_Mill_SimData.RadPerStep)*point.x + sin(m_Mill_SimData.RadPerStep)*point.y;
    	 m_KWorldObject[index].vertex[k].y = -sin(m_Mill_SimData.RadPerStep)*point.x + cos(m_Mill_SimData.RadPerStep)*point.y;
    	 m_KWorldObject[index].vertex[k]   = m_KWorldObject[index].vertex[k] + about; /* Now Translate back */
       }

       /* Update Surfaces based on new vertex positions */
       for(int j=0;j<m_KWorldObject[index].num_surfaces;j++)
       {
    	 sum = make_float3(0.0f,0.0f,0.0f);

         for( int f=0; f<m_KWorldObject[index].surfaces[j].num_vertex; f++ )
         {
	       sum = sum + m_KWorldObject[index].vertex[ m_KWorldObject[index].surfaces[j].vertex_Order[f] ];
         }

         m_KWorldObject[index].surfaces[j].centroid = (1.0f/m_KWorldObject[index].surfaces[j].num_vertex)*sum;

         float3  V1 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[1]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[0]];

         float3  V2 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[2]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[1]];



         /* Compute new normal */
         float3 n         = cross(V2,V1);
         m_KWorldObject[index].surfaces[j].normal = n/length(n);

         if(m_KWorldObject[index].surfaces[j].num_vertex>3)
         {
         float3  V3 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[3]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[2]];


         float3  V4 = m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[0]] -
        		 m_KWorldObject[index].vertex[m_KWorldObject[index].surfaces[j].vertex_Order[3]];

              m_KWorldObject[index].surfaces[j].area= 0.500000f*length(n) + 0.500000f*length(cross(V3,V4));
         }
         else
         {
        	 m_KWorldObject[index].surfaces[j].area= 0.500000f*length(n);
         }


       }

	}/* end world object*/

  }

     /* Update the Device */
     m_KDevice->IDevice_UpdateWorldObjects(m_KWorldObject);
}

/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/
/*        Rotates a Dynamic Object Incrementally(Variable Speed)             */
/*---------------------------------------------------------------------------*/
void Translate_VolObject(int index, float3 Disp)
{
	for(int k=0;k<m_KDynamicObject[index].num_vertex;k++)
	{
       m_KDynamicObject[index].vertex[k]+= Disp;
	}

	float3 sum = make_float3(0.0f,0.0f,0.0f);
     /* Update Surfaces */
     for(int j=0;j<m_KDynamicObject[index].num_faces;j++)
     {
    	 sum = make_float3(0.0f,0.0f,0.0f);

         for(int f=0;f<m_KDynamicObject[index].faces[j].num_vertex;f++)
         {
	       sum = sum + m_KDynamicObject[index].vertex[ m_KDynamicObject[index].faces[j].vertex_Order[f] ];
         }

         m_KDynamicObject[index].faces[j].centroid = (1.0f/m_KDynamicObject[index].faces[j].num_vertex)*sum;

         float3  V1 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[1]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[0]];

         float3  V2 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[2]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[1]];


         float3  V3 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[3]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[2]];


         float3  V4 = m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[0]] -
        		 m_KDynamicObject[index].vertex[m_KDynamicObject[index].faces[j].vertex_Order[3]];

         float3 n         = cross(V2,V1);

         m_KDynamicObject[index].faces[j].normal = n/length(n);
         m_KDynamicObject[index].faces[j].area= 0.500000f*length(n) + 0.500000f*length(cross(V3,V4));

     }


     /* Rotate COM */
     m_KDynamicObject[index].COM   = m_KDynamicObject[index].COM + Disp;


  if(m_OpenGL.render)
  {
     Draw_DynamicGeometry();
  }

     /* Update the Device */
     m_KDevice->IDevice_UpdateVolObjectPositions(m_KDynamicObject);
}


/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*             Rotates a World Object from Start Position                  */
/*---------------------------------------------------------------------------*/
void Translate_WObject(int index, float3 Disp)
{


      float3 sum = make_float3(0.0f,0.0f,0.0f);

      /* Rotate all WObject Vertex about the same point  */
       for(int k=0;k<m_KWorldObject[index].num_vertex;k++)
       {
    	 m_KWorldObject[index].vertex[k]+= Disp;/* Get vertex relative to point */
       }

       /* Update Surfaces based on new vertex positions */
       for(int j=0;j<m_KWorldObject[index].num_surfaces;j++)
       {
    	 sum = make_float3(0.0f,0.0f,0.0f);

         for( int f=0; f<m_KWorldObject[index].surfaces[j].num_vertex; f++ )
         {
	       sum = sum + m_KWorldObject[index].vertex[ m_KWorldObject[index].surfaces[j].vertex_Order[f] ];
         }

         m_KWorldObject[index].surfaces[j].centroid = (1.0f/m_KWorldObject[index].surfaces[j].num_vertex)*sum;

       }


     /* Update the Device */
     m_KDevice->IDevice_UpdateWorldObjects(m_KWorldObject);
}


/*---------------------------------------------------------------------------*/



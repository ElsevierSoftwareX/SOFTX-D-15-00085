/*
 * Graphics.h
 *
 *  Created on: Sep 11, 2012
 *      Author: nicolin
 */


#include <GL/freeglut.h>

#include <math.h>
#include <string>


#ifndef GRAPHICS_H_
#define GRAPHICS_H_

/* Window Size */
const  uint width = 1080, height = 900;


    /* Camera */
int    Gl_ox, Gl_oy;
int    Gl_buttonState        = 0;
float  Gl_camera_trans    [] = {0, 0, -25};
float  Gl_camera_rot      [] = {0, 0, 0};
float  Gl_camera_trans_lag[] = {0, 0, -3};
float  Gl_camera_rot_lag  [] = {0, 0, 0};
float  GL_modelView      [16];


float Vel_Mill;

       /* Flags */

bool Gl_bPause     = true;
bool Gl_DebugLines = false;
bool Gl_Wireframe  = true;
bool Gl_WireframeP = false;

const float Gl_Camera_Inertia = 0.1f;


         /* Lighting */
/* (shiny) component of the material */
GLfloat materialSpecular[] = {0.50f, 0.50f, 0.50f, 1.0f};
/*The color emitted by the material*/
GLfloat materialEmission[] = {0.0, 0.0, 0.0, 1.0f};
/*reflection of the material */
GLfloat shininess=15.0f;
GLfloat Col[10][3];


bool m_is_GLDisplayEvent = false;


/* Simulation Flags */
bool m_resetsim    = false;
bool m_openHatch   = false;
bool m_dynamic_rotation  = false;
bool m_dynamic_translate  = false;
bool m_xImpulse    = false;
bool m_zImpulse    = false;


int m_run_TimeLimit=0;

float PP_Fric[6];

GLfloat Orange[] = {1.0f,0.5f,0.0f};

GLfloat VelCol[60][3];


string CharList[16];


string msg;
GLint MainWindow, DataWindow;


void Draw_DynamicGeometry();
#include "../Input/Processing_Mills.h"
#include "../Input/FileIO.h"

void SetVelColor_Ramp()
{

  VelCol[0][0]=0.0f;
  VelCol[0][1]=0.0f;
  VelCol[0][2]=1.0f;

  VelCol[1][0]=0.0f;
  VelCol[1][1]=1.0f;
  VelCol[1][2]=1.0f;

  VelCol[2][0]=0.0f;
  VelCol[2][1]=1.0f;
  VelCol[2][2]=0.0f;

  VelCol[3][0]=1.0f;
  VelCol[3][1]=1.0f;
  VelCol[3][2]=0.0f;

  VelCol[4][0]=1.0f;
  VelCol[4][1]=0.0f;
  VelCol[4][2]=1.0f;

  VelCol[5][0]=1.0f;
  VelCol[5][1]=0.0f;
  VelCol[5][2]=0.0f;





}


void drawColorBar()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    int w = glutGet( GLUT_WINDOW_WIDTH );
    int h = glutGet( GLUT_WINDOW_HEIGHT );
    glOrtho( 0, w, 0, h, -1, 1 );

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable( GL_DEPTH_TEST );

    glDisable( GL_LIGHTING );
    glColor3f(1, 0, 0);



    for (int i=0;i<6;i++)
    {
    stringstream ss;
    ss<<i*m_OpenGL.color_bin;

     msg = ss.str();

    glColor3f (VelCol[i][0], VelCol[i][1], VelCol[i][2]);
    glRasterPos2f (5+i*25, 20.0);
    glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)msg.c_str() );

    glBegin(GL_QUADS);
      glVertex2f(10.0+i*20,40.0);
      glVertex2f(10.0+i*20,60.0);
      glVertex2f(30.0+i*20,60.0);
      glVertex2f(30.0+i*20,40.0);
    glEnd();

    }

    stringstream ss;
    ss<<" Translational Velocity(m/s)";

   msg = ss.str();

    glColor3f (0.0,0.0, 0.0);
    glRasterPos2f (10, 75.0);
    glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)msg.c_str() );

    glEnable( GL_LIGHTING );

    glEnable (GL_DEPTH_TEST);

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
}


void SetColor_Ramp()
{
	SetVelColor_Ramp();

	Col[0][0] = 0.90;
	Col[0][1] = 0.90;
	Col[0][2] = 0.90;

	Col[1][0] = 0.010;
	Col[1][1] = 0.010;
	Col[1][2] = 0.0;

	Col[2][0] = 0.0;
	Col[2][1] = 0.0;
	Col[2][2] = 1.0;

	Col[3][0] = 1.0;
	Col[3][1] = 1.0;
	Col[3][2] = 0.0;

	Col[4][0] = 0.0;
	Col[4][1] = 1.0;
	Col[4][2] = 1.0;

	Col[5][0] = 1.0;
	Col[5][1] = 0.0;
	Col[5][2] = 1.0;

	Col[6][1] = 1.0;
	Col[6][2] = 1.0;
	Col[6][3] = 1.0;
}

/*---------------------------------------------------------------------------*/
void CreateEnvironment(void)
{

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);

    GLfloat ambientColor[] = {0.0f, 0.2f, 0.2f, 1.0f}; //Color(0.2, 0.2, 0.2)
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);


    //Add directed light
    GLfloat lightColor1[] = {0.5f, 0.2f, 0.2f, 1.0f}; //Color (0.5, 0.2, 0.2)
    GLfloat lightPos1[] = {-1.0f, 0.5f, 0.5f, 0.0f};
    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor1);
    glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);

    GLfloat lightColor2[] = {0.5f, 0.2f, 0.2f, 1.0f}; //Color (0.5, 0.2, 0.2)
    GLfloat lightPos2[] = {20.0f, 30.0f, 0.0f, 0.0f};
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor2);
    glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);




}
/*---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------*/

void ReshapeGL(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

    float aspect = (float)width/(float)height;

    float x = 2.0f*m_KWorldObject[0].cyl_geo.radius*0.70f;
    glOrtho(-x, x, -x/aspect, x/aspect, -x, x);


  glMatrixMode(GL_MODELVIEW);

}

void ReshapeGLP(int w, int h)
{
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

    gluPerspective(60.0, (float) w / (float) h, 0.1, 100.0);


  glMatrixMode(GL_MODELVIEW);

}


/*---------------------------------------------------------------------------*/
void mouse(int button, int state, int x, int y)
{
    int mods;

    if (state == GLUT_DOWN)
    {
        Gl_buttonState |= 1<<button;
    }
    else if (state == GLUT_UP)
    {
        Gl_buttonState = 0;
    }

    mods = glutGetModifiers();

    if (mods & GLUT_ACTIVE_SHIFT)
    {
        Gl_buttonState = 2;
    }
    else if (mods & GLUT_ACTIVE_CTRL)
    {
        Gl_buttonState = 3;
    }

    Gl_ox = x ;
    Gl_oy = y ;


    m_is_GLDisplayEvent=true;
    glutPostRedisplay();
}
/*---------------------------------------------------------------------------*/

        /* transfrom vector by matrix */
void xform(float *v, float *r, GLfloat *m)
{
    r[0] = v[0]*m[0] + v[1]*m[4] + v[2]*m[8] + m[12];
    r[1] = v[0]*m[1] + v[1]*m[5] + v[2]*m[9] + m[13];
    r[2] = v[0]*m[2] + v[1]*m[6] + v[2]*m[10] + m[14];
}
/*---------------------------------------------------------------------------*/

  /* transform vector by transpose of matrix */
void ixform(float *v, float *r, GLfloat *m)
{
    r[0] = v[0]*m[0] + v[1]*m[1] + v[2]*m[2];
    r[1] = v[0]*m[4] + v[1]*m[5] + v[2]*m[6];
    r[2] = v[0]*m[8] + v[1]*m[9] + v[2]*m[10];
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void ixformPoint(float *v, float *r, GLfloat *m)
{
    float x[4];
    x[0] = v[0] - m[12];
    x[1] = v[1] - m[13];
    x[2] = v[2] - m[14];
    x[3] = 1.0f;
    ixform(x, r, m);
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void motion(int x, int y)
{
    float dx, dy;
    dx = (float)(x - Gl_ox);
    dy = (float)(y - Gl_oy);


    if (Gl_buttonState == 3)
    {
      /* left+middle = zoom */
       Gl_camera_trans[2] += (dy / 100.0f) * 0.5f * fabs(Gl_camera_trans[2]);
    }
    else if (Gl_buttonState & 2)
    {
      /* middle = translate */
       Gl_camera_trans[0] += dx / 100.0f;
       Gl_camera_trans[1] -= dy / 100.0f;
    }
    else if (Gl_buttonState & 1)
    {
      /* left = rotate */
       Gl_camera_rot[0] += dy / 5.0f;
       Gl_camera_rot[1] += dx / 5.0f;
     }

    Gl_ox = x;
    Gl_oy = y;

    glutPostRedisplay();
}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
void GL_DisplaySet()
{
      /* Clear Screen */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

   /* Update State matrix to view with camera */
    for (int c = 0; c < 3; ++c)
    {
        Gl_camera_trans_lag[c] += (Gl_camera_trans[c] - Gl_camera_trans_lag[c]) * Gl_Camera_Inertia;
        Gl_camera_rot_lag[c] += (Gl_camera_rot[c] - Gl_camera_rot_lag[c]) * Gl_Camera_Inertia;
    }

    glTranslatef(Gl_camera_trans_lag[0], Gl_camera_trans_lag[1], Gl_camera_trans_lag[2]);
    glRotatef(Gl_camera_rot_lag[0], 1.0, 0.0, 0.0);
    glRotatef(Gl_camera_rot_lag[1], 0.0, 1.0, 0.0);

    glGetFloatv(GL_MODELVIEW_MATRIX, GL_modelView);

}

/*---------------------------------------------------------------------------*/
void normcrossprod(float v1[3], float v2[3], float out[3])
{
  GLint i, j;
  GLfloat length;

  out[0] = v1[1]*v2[2] - v1[2]*v2[1];
  out[1] = v1[2]*v2[0] - v1[0]*v2[2];
  out[2] = v1[0]*v2[1] - v1[1]*v2[0];

}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/

void Draw_DebugLines( float3 centroids, float3 normals )
{

    	glColor3f(1.0,0.0,0.2);
	    glPointSize(4.0);

	    glBegin(GL_POINTS);
	      glVertex3f( centroids.x, centroids.y, centroids.z);
	    glEnd();

	    glLineWidth(8.0f);
	    /* Normal at face centroid */
	    float3 end= centroids + 20.0f*normals;
	    glBegin(GL_LINES);
	      glVertex3f( centroids.x, centroids.y, centroids.z);
	      glVertex3f(end.x,end.y,end.z);
	    glEnd();
	    glLineWidth(1.0f);


}




void Draw_GridLines()
{
  int Nx,Ny,Nz;
  Nx = SimInfo.num_NNCells.x;
  Ny = SimInfo.num_NNCells.y;
  Nz = SimInfo.num_NNCells.z;


  float xpos= SimInfo.worldOrigin.x;
  float ypos= SimInfo.worldOrigin.y;
  float zpos= SimInfo.worldOrigin.z;

 // printf( " GridLines  %f %f %f \n",xpos,ypos,zpos);

  glColor3f(0.0,0.0,0.0);
  glLineWidth((GLfloat)1);


	for( int z=0;z<Nz; z++ )
	{


	for( int x=0;x<Nx; x++ )
	{
  	   glBegin(GL_LINES);
	      glVertex3f( xpos, SimInfo.worldOrigin.y, zpos);
	      glVertex3f( xpos, Ny*SimInfo.cellSize.y, zpos);
	   glEnd();

	   xpos += SimInfo.cellSize.x;

	}
	xpos= SimInfo.worldOrigin.x;
	zpos += SimInfo.cellSize.z;
	}


	 xpos= SimInfo.worldOrigin.x;
	 ypos= SimInfo.worldOrigin.y;
	 zpos= SimInfo.worldOrigin.z;


	/* Draw Y lines */
//	for( int z=0;z<Nz; z++ )
//	{
//
//
//	for( int y=0;y<Ny; y++ )
//	{
//  	   glBegin(GL_LINES);
//	      glVertex3f( SimInfo.worldOrigin.x,ypos , zpos);
//	      glVertex3f( Nx*SimInfo.cellSize.x,ypos, zpos);
//	   glEnd();
//
//	   ypos += SimInfo.cellSize.y;
//
//	}
//	ypos= SimInfo.worldOrigin.y;
//	zpos += SimInfo.cellSize.z;
//	}

}





void Draw_Edges( float3 centroids, float3 normals,int num )
{

    	glColor3f(1.0,0.0,0.2);
	    glPointSize(6.0);

	    glBegin(GL_POINTS);
	      glVertex3f( centroids.x, centroids.y, centroids.z);
	    glEnd();

	    /* Edge normals */
//	    float3 end= centroids + normals;
//	    glLineWidth((GLfloat)4);
//	    glBegin(GL_LINES);
//	      glVertex3f( centroids.x, centroids.y, centroids.z);
//	      glVertex3f(end.x,end.y,end.z);
//	    glEnd();

	    stringstream ss;
	    ss<< num;

	    string msg=ss.str();
        glColor3f (1.0, 0.0, 0.0);
        glRasterPos3f (centroids.x*1.05, centroids.y*1.05, centroids.z*1.05);
        glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)msg.c_str() );
}


/*---------------------------------------------------------------------------*/
                /* Creates the DisplayLists for WorldObjects */
/*---------------------------------------------------------------------------*/

void Draw_WorldGeometry()
{

	SetColor_Ramp();
	glLineWidth(2.4f);

  glNewList(1,GL_COMPILE);
  for( int i=0; i<m_num_KWorldObjects; i++ )
  {

      if(m_KWorldObject[i].surface_type==plane)
      {
	  for( int j=0; j<m_KWorldObject[i].num_surfaces; j++ )
	  {

		   if(m_OpenGL.Debug_Lines==1)
		   {
		     Draw_DebugLines(m_KWorldObject[i].surfaces[j].centroid,m_KWorldObject[i].surfaces[j].normal);
		   }

		  if( m_KWorldObject[i].surfaces[j].num_vertex==3 )
		  {
			glBegin(GL_TRIANGLES);
		  }
		  else
		  {
			if(m_OpenGL.world_surface_Shading[i][j]==false)
			{
			  glBegin(GL_LINE_LOOP);
			}
			else
			{
		      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, Col[i+2]);
		      glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
		      glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
		      glMaterialf(GL_FRONT, GL_SHININESS, shininess);

				glBegin(GL_QUADS);
			}
		  }


	       if(m_OpenGL.color_type==0 || !SimInfo.isMillSim)/* Color by velocity */
	       {


		   GLfloat brown[] = {0.5f,0.35f,0.05f};

		     glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, brown);
		     glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
		     glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
		     glMaterialf(GL_FRONT, GL_SHININESS, 5);
	       }

          glNormal3f( m_KWorldObject[i].surfaces[j].normal.x,
        		      m_KWorldObject[i].surfaces[j].normal.y,
        		      m_KWorldObject[i].surfaces[j].normal.z);

		  for( int k=0; k<m_KWorldObject[i].surfaces[j].num_vertex; k++ )
		  {
			glVertex3f(
			   m_KWorldObject[i].vertex[ m_KWorldObject[i].surfaces[j].vertex_Order[k] ].x,
			   m_KWorldObject[i].vertex[ m_KWorldObject[i].surfaces[j].vertex_Order[k] ].y,
			   m_KWorldObject[i].vertex[ m_KWorldObject[i].surfaces[j].vertex_Order[k] ].z);
		  }
		    glEnd();
	    }
      }
      else
      {


    	 float thickness=m_KWorldObject[0].cyl_geo.radius*1.025f -m_KWorldObject[0].cyl_geo.radius;

    	 /* Draw caps */
    	  glColor3f(0.2f,0.07f,0.06f);
    	    glPushMatrix();
    	      GLUquadricObj *capb=NULL;
    	      capb = gluNewQuadric();
    	      glTranslatef(m_KWorldObject[0].cyl_geo.center_bot_cap.x, m_KWorldObject[0].cyl_geo.center_bot_cap.y,
    	     		    	    		                                m_KWorldObject[0].cyl_geo.center_bot_cap.z );

    	     gluDisk(capb,m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.radius+thickness,120,120);
    	    glPopMatrix();

    	    glColor3f(0.2f,0.07f,0.06f);
      	    glPushMatrix();
      	      GLUquadricObj *capt=NULL;
      	      capt = gluNewQuadric();
      	      glTranslatef(m_KWorldObject[0].cyl_geo.center_top_cap.x, m_KWorldObject[0].cyl_geo.center_top_cap.y,
      	     		    	    		                                m_KWorldObject[0].cyl_geo.center_top_cap.z );
      	      gluDisk(capt,m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.radius+thickness,120,120);
      	    glPopMatrix();
           /* End Draw Caps */


      	  glColor3f(0.2f,0.07f,0.06f);
    	  glPushMatrix();
    		GLUquadricObj *cyl=NULL;
    		cyl = gluNewQuadric();
    	    glTranslatef(m_KWorldObject[0].cyl_geo.center_bot_cap.x, m_KWorldObject[0].cyl_geo.center_bot_cap.y,
    	    		                                                 m_KWorldObject[0].cyl_geo.center_bot_cap.z);
    		gluCylinder( cyl,m_KWorldObject[0].cyl_geo.radius,
    				m_KWorldObject[0].cyl_geo.radius,
    				m_KWorldObject[0].cyl_geo.height,120,120 );


    	  glPopMatrix();


    	  /* Dummy outside cylinder */
      	  glColor3f(0.2f,0.07f,0.06f);
    	  glPushMatrix();
    		GLUquadricObj *cylo=NULL;
    		cylo = gluNewQuadric();
    	    glTranslatef(m_KWorldObject[0].cyl_geo.center_bot_cap.x, m_KWorldObject[0].cyl_geo.center_bot_cap.y,
    	    		                                                 m_KWorldObject[0].cyl_geo.center_bot_cap.z);
    		//glRotatef(-90.0f, 1.0f, 0.0f,0.0f);
    		gluCylinder( cylo,m_KWorldObject[0].cyl_geo.radius*1.025,
    				m_KWorldObject[0].cyl_geo.radius*1.025f,
    				m_KWorldObject[0].cyl_geo.height,80,80 );
    	  glPopMatrix();


    	  /* Draw Time lines*/

    		  glColor3f(0.f,0.0f,0.0f);
    		    glLineWidth((GLfloat)8);
    		    glBegin(GL_LINES);
    		      glVertex3f(m_KWorldObject[0].cyl_geo.radius, -m_KWorldObject[0].cyl_geo.radius*0.025, m_KWorldObject[0].cyl_geo.height/2.0);
    		      glVertex3f(m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.radius*2.025, m_KWorldObject[0].cyl_geo.height/2.0);
    		    glEnd();


      		  glColor3f(0.f,0.0f,0.0f);
      		    glLineWidth((GLfloat)8);
      		    glBegin(GL_LINES);
      		      glVertex3f(-m_KWorldObject[0].cyl_geo.radius*0.025, m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.height/2.0);
      		      glVertex3f(m_KWorldObject[0].cyl_geo.radius*2.025, m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.height/2.0);
      		    glEnd();




      }

  }


  if (SimInfo.Rotating_WObject)
  {

     	 m_KWorldObject[0].cyl_geo.center_bot_cap = make_float3(m_KWorldObject[0].cyl_geo.radius,m_KWorldObject[0].cyl_geo.radius,0.0f);
     	 m_KWorldObject[0].cyl_geo.height = 20.0f;



   	  glColor3f(0.2f,0.07f,0.06f);
 	  glPushMatrix();
 		GLUquadricObj *cyl=NULL;
 		cyl = gluNewQuadric();
 	    glTranslatef(m_KWorldObject[0].cyl_geo.center_bot_cap.x, m_KWorldObject[0].cyl_geo.center_bot_cap.y,
 	    		                                                 0.0);
 		gluCylinder( cyl,m_KWorldObject[0].cyl_geo.radius,
 				m_KWorldObject[0].cyl_geo.radius,
 				m_KWorldObject[0].cyl_geo.height,120,120 );
 	  glPopMatrix();

	  /* Draw Time lines*/

		  glColor3f(0.f,0.0f,0.0f);
		    glLineWidth((GLfloat)8);
		    glBegin(GL_LINES);
		      glVertex3f(m_KWorldObject[0].cyl_geo.radius, -m_KWorldObject[0].cyl_geo.radius*0.025, 0.0);
		      glVertex3f(m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.radius*2.025, 0.0);
		    glEnd();


  		  glColor3f(0.f,0.0f,0.0f);
  		    glLineWidth((GLfloat)8);
  		    glBegin(GL_LINES);
  		      glVertex3f(-m_KWorldObject[0].cyl_geo.radius*0.025, m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.height/2.0);
  		      glVertex3f(m_KWorldObject[0].cyl_geo.radius*2.025, m_KWorldObject[0].cyl_geo.radius, m_KWorldObject[0].cyl_geo.height/2.0);
  		    glEnd();

  }

   //printf(" debug level %d \n",m_OpenGL.Debug_Lines);
   if(m_OpenGL.Debug_Lines==4)
   {
    Draw_GridLines();
   }

  glEndList();
//printf(" World drawn\n");

}





/*---------------------------------------------------------------------*/
              /* Creates the DisplayLists for Particles */
/*---------------------------------------------------------------------------*/
void Draw_ParticleGeometry_Sphere()
{
  SetColor_Ramp();

//if(m_OpenGL.Debug_Lines==1)
//{
//	Gl_WireframeP = true;
//}

  for( int i=0; i<m_num_KParticleObjects; i++ )
  {
	 glNewList(10+i,GL_COMPILE);

    if(m_OpenGL.color_type==0)
    {
	 glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, Col[i]);
	 glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
	 glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
	 glMaterialf(GL_FRONT, GL_SHININESS, shininess);
    }
	      glPushMatrix();

		  glutSolidSphere(m_KParticleObject[i].radius,20,20);

		  if(Gl_WireframeP)
		  {

				 glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, Col[6]);
				 glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
				 glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
				 glMaterialf(GL_FRONT, GL_SHININESS, shininess);
		    glutWireSphere(m_KParticleObject[i].radius*1.02f,15,15);
		  }
		  glPopMatrix();

	 glEndList();

  }


}





/*---------------------------------------------------------------------*/
              /* Creates the DisplayLists for Particles */
/*---------------------------------------------------------------------------*/
void Draw_ParticleGeometry_Poly()
{


  for( int i=0; i<m_num_KParticleObjects; i++ )
  {


	 glNewList(10+i,GL_COMPILE);


	 if(m_OpenGL.color_type==0)
	 {
      glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, Col[i]);
      glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
      glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
      glMaterialf(GL_FRONT, GL_SHININESS, shininess);

	  }

	 glPushMatrix();
  	    for( int j=0; j<m_KParticleObject[i].num_faces; j++ )
	    {
		   if( m_KParticleObject[i].face[j].num_vertex==3 )
		   {
			 glBegin(GL_TRIANGLES);
		   }
		   else if( m_KParticleObject[i].face[j].num_vertex==4 )
		   {
			 glBegin(GL_QUADS);
		   }
		   else
		   {
			   glBegin(GL_POLYGON);
		   }


               glNormal3f( m_KParticleObject[i].face[j].normal.x,
        	 	           m_KParticleObject[i].face[j].normal.z,
        		           m_KParticleObject[i].face[j].normal.y);

		       for( int k=0; k<m_KParticleObject[i].face[j].num_vertex; k++ )
		       {

			      float3 vertex = m_KParticleObject[i].vertex
			    		     [ m_KParticleObject[i].face[j].vertex_Order[k] ];

			      glVertex3f(vertex.x,vertex.y,vertex.z);
		        }
		     glEnd();

		     if(m_OpenGL.Debug_Lines==3)
		     {
		      Draw_DebugLines(m_KParticleObject[i].face[j].centroid,m_KParticleObject[i].face[j].normal);
		     }

	    }/* End looping over faces */

      /* End Drawing polyhedara */

  //		/* Draw edges */

	  if(m_OpenGL.Debug_Lines==5)
	  {
		for(int j=0; j<m_KParticleObject[i].num_edges; j++)
		{
		  float3 Point = m_KParticleObject[i].vertex[m_KParticleObject[i].edge[j].point_vindex[0]];
		  float3 Dir =  (m_KParticleObject[i].vertex[m_KParticleObject[i].edge[j].point_vindex[1]]-Point);


		  float3 cent = (Point + m_KParticleObject[i].vertex[m_KParticleObject[i].edge[j].point_vindex[1]])*0.5f;

		  Draw_Edges(cent,Dir,j);

		  glPointSize(8.0);
            glBegin( GL_POINTS );
		      glVertex3f( cent.x, cent.y,cent.z );
		  	glEnd();

//		  	float3 normal = (m_KParticleObject[i].face[m_KParticleObject[i].edge[j].normal_sindex[0]].normal +
//		                    m_KParticleObject[i].face[m_KParticleObject[i].edge[j].normal_sindex[1]].normal);
//		  	normal = normal/length(normal);
//
//		  	Draw_DebugLines(cent, normal );
		}
	  }

  	  glPopMatrix();
  	  glEndList();
  }





}


void Draw_DynamicGeometry()
{

  for( int i=0; i<m_num_KDynamicObjects; i++ )
  {

	 glNewList(50+i,GL_COMPILE);


     if(m_OpenGL.Debug_Lines==2)
     {

 	  glColor3f(0.2f,0.07f,0.06f);
	  glPushMatrix();
		GLUquadricObj *cyl=NULL;
		cyl = gluNewQuadric();
	    glTranslatef(m_KDynamicObject[i].COM.x, m_KDynamicObject[i].COM.y,
	    		m_KDynamicObject[i].COM.z);

	    if (m_KDynamicObject[i].Axis.x==0.0f)
	    {
	      glRotatef(90.0f,0.0f,1.0f,0.0f);
	    }
	    if (m_KDynamicObject[i].Axis.y==0.0f)
	    {
	      glRotatef(90.0f,0.0f,0.0f,1.0f);
	    }

		gluCylinder( cyl,m_KDynamicObject[i].boundR, m_KDynamicObject[i].boundR, 5.0,120,120 );
	  glPopMatrix();

     }

  	    for( int j=0; j< m_KDynamicObject[i].num_faces; j++ )
	    {

		     if(m_OpenGL.Debug_Lines==2)
		     {
  	    	   Draw_DebugLines( m_KDynamicObject[i].faces[j].centroid, m_KDynamicObject[i].faces[j].normal);
		     }

		   if( m_KDynamicObject[i].faces[j].num_vertex==3 )
		   {
			 glBegin(GL_TRIANGLES);
		   }
		   else
		   {
			 glBegin(GL_QUADS);
		   }



		     glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, Orange);
		     glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
		     glMaterialfv(GL_FRONT, GL_EMISSION, materialEmission);
		     glMaterialf(GL_FRONT, GL_SHININESS, 5);

		        if( i==0)
		        {
		         glColor3f(0.0f,1.0f,0.0f);
		       }
               glNormal3f( m_KDynamicObject[i].faces[j].normal.x,
        	 	           m_KDynamicObject[i].faces[j].normal.z,
        		           m_KDynamicObject[i].faces[j].normal.y);

		       for( int k=0; k<m_KDynamicObject[i].faces[j].num_vertex; k++ )
		       {

			      float3 vertex = m_KDynamicObject[i].vertex
			    		     [ m_KDynamicObject[i].faces[j].vertex_Order[k] ];

			      glVertex3f(vertex.x,vertex.y,vertex.z);
		        }
		     glEnd();

	    }/* End looping over faces */


  	    /* Draw Dummy Ends */
         if(SimInfo.isMillSim)
         {
			 glBegin(GL_QUADS);

             glNormal3f(0.0,0.0,1.0f);


		       glColor3f(0.1f,0.07f,0.06f);

		       if(i==0)
		       {
		    	   glColor3f(0.0f,1.0f,0.0f);
		       }

			      glVertex3f(m_KDynamicObject[i].vertex[0].x,m_KDynamicObject[i].vertex[0].y,m_KDynamicObject[i].vertex[0].z);
			      glVertex3f(m_KDynamicObject[i].vertex[3].x,m_KDynamicObject[i].vertex[3].y,m_KDynamicObject[i].vertex[3].z);

			      glVertex3f(m_KDynamicObject[i].vertex[7].x,m_KDynamicObject[i].vertex[7].y,m_KDynamicObject[i].vertex[7].z);


			      glVertex3f(m_KDynamicObject[i].vertex[4].x,m_KDynamicObject[i].vertex[4].y,m_KDynamicObject[i].vertex[4].z);

		     glEnd();


		  	    /* Draw Dummy Ends */

					 glBegin(GL_QUADS);
					 glNormal3f(0.0,0.0,1.0f);

				       glColor3f(0.1f,0.07f,0.06f);

				       if(i==0)
				       {
				    	   glColor3f(0.0f,1.0f,0.0f);
				       }

					      glVertex3f(m_KDynamicObject[i].vertex[2].x,m_KDynamicObject[i].vertex[2].y,m_KDynamicObject[i].vertex[2].z);
					      glVertex3f(m_KDynamicObject[i].vertex[1].x,m_KDynamicObject[i].vertex[1].y,m_KDynamicObject[i].vertex[1].z);

					      glVertex3f(m_KDynamicObject[i].vertex[5].x,m_KDynamicObject[i].vertex[5].y,m_KDynamicObject[i].vertex[5].z);
					      glVertex3f(m_KDynamicObject[i].vertex[6].x,m_KDynamicObject[i].vertex[6].y,m_KDynamicObject[i].vertex[6].z);


				     glEnd();
         }

	 glEndList();


  }


}




void displayAxis()
{
         // Axis
	     glLineWidth(3.0f);
         glBegin (GL_LINES);
         glColor3f (1.0, 0.0, 0.0);
         glVertex3i (0, 0, 0);
         glVertex3i (1, 0, 0);   // X
         glColor3f (1.0, 0.0, 0.0);
         glVertex3i (0, 0, 0);
         glVertex3i (0, 1, 0);   // Y
         glColor3f (0.0, 1.0, 0.0);
         glVertex3i (0, 0, 0);
         glVertex3i (0, 0, 1);   // Z
         glEnd ();

         glLineWidth(1.0f);

         // Text Axis
         glColor3f (1.0, 0.0, 0.0);
         glRasterPos3f (1.2, 0.0, 0.0);
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_18, 'X');
         glColor3f (1.0, 0.0, 0.0);
         glRasterPos3f (0.0, 1.2, 0.0);
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_18, 'Y');
         glColor3f (0.0, 1.0, 0.0);
         glRasterPos3f (0.0, 0.0, 1.2);
         glutBitmapCharacter (GLUT_BITMAP_HELVETICA_18, 'Z');
}










void keyboard(unsigned char key,int,int)
{

    switch (key)
    {
        case ' ':
            Gl_bPause = !Gl_bPause;
            break;


        case 'o':

        	if(!m_Silo_SimData.is_hatch_open)
        	{
        	  m_KDevice->IDevice_OpenHatch(m_KWorldObject,m_num_KWorldObjects-1);
        	  m_Silo_SimData.is_hatch_open=true;

        	  if(SimInfo.Simulation_Type==silo)
        	  {
        	    printf("Hatch opened at: %f (seconds) \n",simtime_elapsed );

        	  }
        	  else
        	  {
        		 m_num_KWorldObjects--;
        		 SimInfo.Num_WorldObjects--;
        	  }
        	}
        	simtime_elapsed=0.0f;
        break;


        case 's':
                m_dynamic_rotation = !m_dynamic_rotation;

                if(m_dynamic_rotation)
                {
                	printf("Rotation started: \n" );
                	m_KDevice->IDevice_drum_rotation(true);
                }
                else
                {
                	printf("Rotation stopped: \n" );
                	m_KDevice->IDevice_drum_rotation(false);
                }

                if(m_Current_StepNumber==0 && SimInfo.EnergyCalc)
                {
                  printf("Reset Energy: \n" );
                  m_KDevice->IDevice_Mill_reset_energy(0.0f);
                }



        break;


        case 'p':

        	 Output_SystemState(user,0,3);
        	 printf("Output to file: Done! \n" );

        	break;

        case 'c':

        	Output_SystemState(user,0,5);
        	printf("Output to file (Point cloud): Done! \n" );


        break;

        case 'q':
        	m_KDevice->clean();
        	  delete [] m_d_P_ID;
        	  delete [] m_d_Particle_Pos;
        	  delete [] m_d_Particle_Quart;
        	  delete [] m_d_Particle_Type;
        	  delete [] m_colorP;
        	  LogFile.close();



              exit(0);
        break;

        case '+':
        	//m_KDevice->IDevice_AddForce();
        	m_Mill_SimData.RadPerStep +=0.10f*m_Mill_SimData.RadPerStep;
        break;

        case '-':
        	//m_KDevice->IDevice_SubForce();
        	 m_Mill_SimData.RadPerStep -=0.10f*m_Mill_SimData.RadPerStep;
        break;

        case 'z':
        	m_KDevice->IDevice_ZeroForce();
        break;

        case 'w':
        	Gl_camera_trans[2] += 0.50f*fabs(Gl_camera_trans[2]);
        break;

        case 'e':
        	Gl_camera_trans[2] -= 0.50f* fabs(Gl_camera_trans[2]);
        break;

        case 'j':
        	Translate_VObj = true;
        break;

        case 'k':
        	Translate_VObj = false;
        break;



    }
}






#endif /* GRAPHICS_H_ */

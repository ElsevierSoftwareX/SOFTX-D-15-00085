/* Contains the Kernels and methods that perform
 * Spatial sub-division of the problem  */


/*-----------------------------------------------------------------------------*/
                     /* DEVICE COMPUTING METHODS */
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
                      /* Given a Point: p
                       * Returns it position in the
                       * computational grid */

/* !!Cost per Call!!
 *    Storage       : 6 Bytes
 *    Memory        : 6 Reads (Const)
 *    Computational : 3 Sub,3 Div, 1 Floor */
/*-----------------------------------------------------------------------------*/
__device__ int3 SpatialDecomp_CalcGridPos(float3 p)
{
    int3 gridPos;

    gridPos.x = floor((p.x - SimParms.worldOrigin.x) / SimParms.cellSize.x);
    gridPos.y = floor((p.y - SimParms.worldOrigin.y) / SimParms.cellSize.y);
    gridPos.z = floor((p.z - SimParms.worldOrigin.z) / SimParms.cellSize.z);

    return gridPos;
}
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
                     /* Given a grid position
                      * Returns a Hash based on the spatial location */
/* !!Cost per Call!!
 *    Storage       : 0 Bytes
 *    Memory        : 3 Reads (Const)
 *    Computational : 3 Add, 3 Mul */

/*-----------------------------------------------------------------------------*/
__device__ uint SpatialDecomp_CalcGridHash(int3 gridPos)
{

    return ( (gridPos.x + gridPos.z*SimParms.num_NNCells.z) +
			 (gridPos.y*SimParms.num_NNCells.x)*SimParms.num_NNCells.z);



}
/*-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------*/
                       /* GLOBAL Kernels */
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
         /* Calculate grid hash value for all particles */
         /* 1) Input   : numberParticles
          * 2) Returns : Hash of each particle : gridParticleHash_R
          * 3) Returns : Index of each particle: gridParticleIndex_R
          * This linked list allows sorting to get better hit rate */


/* !!Cost per thread!!
 *    Storage       : 9 + 6 Bytes
 *    Memory        : Reads:# 1 (global-device) 9 (constant), Writes:# 2 (global-malloc)
 *    Computational : ADD:# 1+3-3, MUL:# 1+3, DIV:# +1,FLOOR:# +1 */
/* Constant Memory IO */
/*-----------------------------------------------------------------------------*/

__global__ void Kernel_SpatialDecomp_CalcPHash( uint   *gridParticleHash,
		                                        int    *gridParticleIndex,
		                                        float3 *position_com,
		                                        uint   Num_Particles)

{
	uint index = blockIdx.x*blockDim.x  + threadIdx.x;



    if (index < Num_Particles )/* stop at max_index-1 */
    {

    	//printf("%d %f \n",index,position_com[index].x );

    	int3 GridPos = SpatialDecomp_CalcGridPos( position_com[index]);

    /* store grid hash and particle index */
      gridParticleHash [index]   = SpatialDecomp_CalcGridHash( GridPos);
      gridParticleIndex[index]   = index;



    }

}
/*-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------*/
             /*   Uses Thrust Sort Method(uses existing device arrays)
                * 1) Ascending based on: dGridParticleHash
                * 2) Linked Sorted     : dGridParticleIndex  */
/*-----------------------------------------------------------------------------*/
void SpatialDecomp_ThrustSort_ByHash( uint *dGridParticleHash, int *dGridParticleIndex,
		                                                       uint numParticles        )
{
    thrust::stable_sort_by_key( thrust:: device_ptr<uint>(dGridParticleHash),
                                thrust:: device_ptr<uint>(dGridParticleHash + numParticles),
                                thrust:: device_ptr<int>(dGridParticleIndex));

}
/*-----------------------------------------------------------------------------*/



/*TODO Cannot sort 2D array*/

/*-----------------------------------------------------------------------------*/
/* Note we keep history between particles and lifter for smooth edge contact */
__global__ void Kernel_SortArrays   ( int      *dGridParticleIndex,
					                  float3    *SortedPos_R,
					                  Quaterion *SortedPos_RQ,
					                  float3    *SortedVel_R,
					                  float3    *SortedVel_RQ,
					                  float3    *SortedAcc_R,
					                  uint      *SortedPType_R,
					                  int       *SortedPID_R,

					                  Contact_Info      *SortedLifter_Contact_Hist_R,
					                  Contact_InfoPP    *SortedPP_Contact_Hist_R,
					                  int              *SortedPP_Contact_Num_R,

									  float3    *SortedPosOld_R,

									  float3    *position_com,
									  Quaterion *position_ornt,
									  float3    *velocity_com,
									  float3    *velocity_ang,
									  float3    *accelrat_com,
									  uint      *P_ObjectType,
									  int       *P_ID,
									  Contact_Info     *Lifter_Contact_Hist,

									  Contact_InfoPP   *PP_Contact_Hist_R,
					                  int              *PP_Contact_Num_R,

									  float3    *position_com_Old,
									  uint   Num_Particles)
{
    uint index = blockIdx.x*blockDim.x + threadIdx.x;

    if (index < Num_Particles )/* stop at max_index-1 */
    {

      /* dGridParticleIndex contains the address of the element in
          * the Particle Dynamics Array (PDA) */

       /* Get the sorted particle address from PDA */
       uint   sortedIndex = dGridParticleIndex[index];

       /* Create a sorted PDA in temp storage as we cant do a
        * direct assignment (similar to temp when sorting)  */
       SortedPos_R   [index]  = position_com  [sortedIndex];
       SortedVel_R   [index]  = velocity_com  [sortedIndex];
       SortedAcc_R   [index]  = accelrat_com  [sortedIndex];

       if(SimParms.particle_type==1 || SimParms.sphere_orient)
       {
         SortedPos_RQ  [index]  = position_ornt [sortedIndex];
       }

       if(SimParms.Integration_Type==1)
       {
         SortedPosOld_R  [index]  = position_com_Old [sortedIndex];
       }

       SortedVel_RQ  [index]  = velocity_ang  [sortedIndex];
       SortedPType_R [index]  = P_ObjectType  [sortedIndex];
       SortedPID_R   [index]  = P_ID          [sortedIndex];

       SortedLifter_Contact_Hist_R    [index] = Lifter_Contact_Hist   [sortedIndex];

       if(SimParms.use_hist)
       {
         SortedPP_Contact_Hist_R    [index] = PP_Contact_Hist_R   [sortedIndex];
         SortedPP_Contact_Num_R    [index] = PP_Contact_Num_R   [sortedIndex];
       }
    }

}
/*-----------------------------------------------------------------------------*/




/*---------------------------------------------------------------------------*/
               /* Updates PDA will sorted list to
                * make memory accesses more efficient
                * 1) Input: All previously allocated*/
/* !!Cost per thread!!
 *    Storage       : 2 Bytes
 *    Memory        : Reads:# 4 (global-malloc), Writes:# 4 (global-device)
 *    Computational : ADD:# 6-1, MUL:# 1,
 *    */
/*---------------------------------------------------------------------------*/
__global__ void Kernel_SpatialDecomp_ReorderPDA( float3 *SortedPos,    Quaterion *SortedPosQ,
		                                         float3 *SortedVel,    float3 *SortedVelQ,
		                                         float3 *SortedAcc,
		                                         uint   *SortedPType,  int *SortedPID,
		                                         Contact_Info   *SortedLifter_Contact_Hist,
		                                         Contact_InfoPP   *SortedPP_Contact_Hist,
		                                         int              *SortedPP_Contact_Num,

		                                         float3 *SortedPosOld,

		                                         float3 *position_com, Quaterion *position_ornt,
		                                         float3 *velocity_com, float3 *velocity_ang,
		                                         float3 *accelrat_com,
		                                         uint   *P_ObjectType, int *P_ID,
		                                         Contact_Info   *Lifter_Contact_Hist,
		                                         Contact_InfoPP   *PP_Contact_Hist,
		                                         int              *PP_Contact_Num,


		                                         float3 *position_com_Old,
		                                         uint   Num_Particles)
{
   uint index = blockIdx.x*blockDim.x  + threadIdx.x ;

   if( index < Num_Particles )
   {

	 /* update */
     position_com  [index] = SortedPos   [index];
     velocity_com  [index] = SortedVel   [index];
     accelrat_com  [index] = SortedAcc   [index];

     if(SimParms.particle_type==1 || SimParms.sphere_orient )
     {
       position_ornt [index] = SortedPosQ  [index];
     }

     if(SimParms.Integration_Type==1 )
     {
       position_com_Old [index] = SortedPosOld  [index];
     }

     velocity_ang [index] = SortedVelQ  [index];

     P_ObjectType [index] = SortedPType [index];
     P_ID         [index] = SortedPID   [index];

     Lifter_Contact_Hist    [index] = SortedLifter_Contact_Hist[index];

     if(SimParms.use_hist)
     {
       PP_Contact_Hist    [index] = SortedPP_Contact_Hist[index];
       PP_Contact_Num    [index] = SortedPP_Contact_Num[index];
     }
   }

}

/*---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
               /* Bins particles into Grid Cells based on the Hash
                * and reorders based on sorting
                * 1) Input  : dGridParticleHash, dGridParticleIndex
                * 2) Output : Cell bin address: cellStart_R, cellEnd_R
                * 3) Output : Sorted Dynamics Info */
/*-----------------------------------------------------------------------------*/
__global__ void Kernel_SpatialDecomp_BinData( uint *dGridParticleHash,
		                                      uint *cellStart_R, uint *cellEnd_R,
		                                      uint   Num_Particles)
{
    uint index = blockIdx.x*blockDim.x + threadIdx.x;


    if (index < Num_Particles )/* stop at max_index-1 */
    {


      /* index gives the index of a particle in the array */
      uint hash;
      uint nexthash;

      hash    = dGridParticleHash[index];

      if ( (index < Num_Particles-1) )/* stop at max_index-1 */
      {
         /* load next hash in list */
         nexthash = dGridParticleHash[index+1];

         /* First entry so cell start */
         if(index==0)
         {
           cellStart_R[hash] = index;
         }

         /* if the hashes are different its a new cell*/
         if (hash != nexthash)
         {
    	   /* End of will be at address (index+1) in sorted List */
    	   cellEnd_R[hash]       = index+1;
    	   /* Start will be at address (index+1) in sorted List */
    	   cellStart_R[nexthash] = index+1;
         }

      }
        /* Last particle must be the cell end */
      if ( index == SimParms.Num_Particles - 1 )
      {
         cellEnd_R[hash] = index + 1;
      }

    }

}
/*-----------------------------------------------------------------------------*/





/*-----------------------------------------------------------------------------*/
               /* 1) Check if two spheres are colliding  */
/*-----------------------------------------------------------------------------*/
__device__ NN_Selected_Cell CollisionDetection_Filter_SphereNNCell(
		                                                      int3   NNgridPos,
		                                                      uint   index,
                                                              uint   *cellStart,
                                                              uint   *cellEnd,
                                                              float3 *position_com,
                                                              uint   *P_ObjectType )
{
    uint gridHash = SpatialDecomp_CalcGridHash(NNgridPos);

    /* get start of bucket for this cell */
    uint startIndex = cellStart[gridHash];

     NN_Selected_Cell Selected_mem_index;

    int num_sel=0;

    /* check cell is not empty */
    if (startIndex != 0xffffffff)
    {


        /* Get end of the cell */
        uint endIndex = cellEnd[gridHash];

        /* Get the COM Position of Current Particle */

        float3 PosA = position_com[index];

        /* Get the Bound Radius Current Particle Type */
        float  R_A = ParticleObject[P_ObjectType[index]].radius;


        /* Loop over all entries  in the cell and compute NN */
        for (uint j=startIndex; j<endIndex; j++)
        {

           if (j != index) /* check not colliding with self */
            {

               /* Check to see if there is an overlap */
               if( length(PosA-position_com[j]) - ( R_A + ParticleObject[ P_ObjectType[j]].radius) < 0.0 )
               {
                  Selected_mem_index.NN_Index[num_sel+1] = j; /* mem index of neighbour */
                  num_sel++;
               }

            }

        }/* End loop over Current cell */

    }/* End checking cell */

    /* First value is how many selected */
    Selected_mem_index.NN_Index[0] = num_sel;

    return Selected_mem_index;
}
/*-----------------------------------------------------------------------------*/




/*-----------------------------------------------------------------------------*/
       /* Broad Phase Collision Test1: Bounding Sphere Distance
        * Stores List of NN for each Particle, checks in parallel */
/*-----------------------------------------------------------------------------*/
__global__ void Kernel_BroadCollisionDetection_NonSymmetry(
                          uint   *cellStart,
                          uint   *cellEnd ,
                          uint   *Broad_List,
                          uint   *NN_NUM,
                          float3 *position_com,
                          uint   *p_ObjectID,
                          int    *P_ID,
                          uint   Num_Particles)
{

	/* get the unique memory location index  */
    uint mem_index = blockIdx.x*blockDim.x + threadIdx.x;

   if( ( mem_index < Num_Particles) && P_ID[mem_index] > -1 )
   {

    /* Read particle pos from sorted arrays */
    float3 pos = position_com[mem_index];

    /* Get hashed address in grid */
    int3 gridPos = SpatialDecomp_CalcGridPos(pos);

    /* examine neighboring cells */
    /* check NN -1 : +1 */

    int zstart = -1;
    int zend   =  1;
    int ystart = -1;
    int yend   =  1;
    int xstart = -1;
    int xend   =  1;

    if(gridPos.x==0)
    {
      xstart = 0;
    }
    if(gridPos.y==0)
    {
      ystart = 0;
    }
    if(gridPos.z==0)
    {
      zstart = 0;
    }

    if(gridPos.x==(SimParms.num_NNCells.x-1))
    {
      xend = 0;
    }
    if(gridPos.y==(SimParms.num_NNCells.y-1))
    {
      yend = 0;
    }
    if(gridPos.z==(SimParms.num_NNCells.z-1))
    {
      zend = 0;
    }

    NN_Selected_Cell Selected_ParticleMemIndex;

    int counter=0;

    /* Search through each of the NN Cells */
    for (int y = ystart; y <= yend; y++)
    {
        for (int z = zstart; z <= zend ; z++)
        {
            for (int x=  xstart; x <= xend; x++)
            {
                int3 neighbourPos = gridPos + make_int3(x, y, z);

                Selected_ParticleMemIndex =
                		CollisionDetection_Filter_SphereNNCell( neighbourPos, mem_index,cellStart, cellEnd, position_com, p_ObjectID);


                for(int i=0; i <Selected_ParticleMemIndex.NN_Index[0];i++)
                {

                  Broad_List[mem_index*32 + (counter+i)]= Selected_ParticleMemIndex.NN_Index[i+1];

                }
                counter = counter + Selected_ParticleMemIndex.NN_Index[0]; /*location 0 is num being returned*/

            }

        }
    }

    NN_NUM[mem_index] = counter;

//    atomicAdd(&TotalNN,counter);
//
//    __syncthreads();
//
//    if (mem_index==0)
//    {
//    	printf("total contacts %d\n",TotalNN);
//    	TotalNN=0;
//    }
    if( counter > 32 )
    {
      printf("ERROR: MAX num neighbors(32) exceeded \n");
      /* Throw an Exception when supported by Device */
    }

//    printf("Pos %f %f %f  Grid Pos %d %d %d  : Particle %d has %d NN \n ", position_com[index].x, position_com[index].y, position_com[index].z,
//  		  gridPos.x,gridPos.y,gridPos.z,  P_ID[index],NN_NUM[index]);

  }/* thread check */

}











                          /* Need to verify this */

/*-----------------------------------------------------------------------------*/
               /* 1) Check if two spheres are colliding  */
/*-----------------------------------------------------------------------------*/
__device__ NN_Selected_Cell CollisionDetection_Filter_2SphereSameCell( int3 OwngridPos,    uint index,
									 uint *cellStart, uint *cellEnd,float3 *position_com,uint *P_ObjectType, int *P_ID)
{
    uint gridHash = SpatialDecomp_CalcGridHash(OwngridPos);

    /* get start of bucket for this cell */
    uint startIndex = cellStart[gridHash];
    /* Get end of the cell */
    uint endIndex = cellEnd[gridHash];


     NN_Selected_Cell Selected_ID;

    int num_sel=0;
   // float delta = SimParms.InitalDelta_t;


   /* Only if we are the first particle in the cell */
   int smallest= SimParms.Num_Particles+1;

   for(uint j=startIndex; j<endIndex; j++)
   {
      if (P_ID[j]<smallest)
      {
         smallest =P_ID[j];
       }
    }

    if(P_ID[index]==smallest)
    {
        /* Get the COM Position of Current Particle */

        float3 PosA = position_com[index];

        /* Get the Bound Radius Current Particle Type */
        float  R_A = ParticleObject[P_ObjectType[index]].radius;


        /* Loop over all entries  in the cell and compute NN */
        for (uint j=startIndex; j<endIndex; j++)
        {

           if (j != index) /* check not colliding with self */
            {

                  /* Check to see if there is an overlap */
                  if( length( PosA - position_com[j] ) - (R_A+ParticleObject[ P_ObjectType[j]].radius) < 0.0 )
                  {
                	  Selected_ID.NN_Index[num_sel+1] = j;/* index of neighbour */
                      num_sel++;
                  }

            }

        }/* End loop over Current cell */
    	}
    /* First value is how many selected */
    Selected_ID.NN_Index[0] = num_sel;

    return Selected_ID;
}
/*-----------------------------------------------------------------------------*/



__device__ NN_Selected_Cell CollisionDetection_Filter_SphereSameCellMany( int3 OwngridPos,    uint index,
									 uint *cellStart, uint *cellEnd,float3 *position_com,uint *P_ObjectType, int *P_ID)
{

NN_Selected_Cell  Selected_ID;
int   num_sel=0;
//float delta = SimParms.InitalDelta_t;

uint gridHash = SpatialDecomp_CalcGridHash(OwngridPos);

    /* Get end of the cell */
uint endIndex = cellEnd[gridHash];


/* Get the COM Position of Current Particle */
float3 PosA = position_com[index];// + delta*velocity_com[index] ;
/* Get the Bound Radius Current Particle Type */
float R_A = ParticleObject[P_ObjectType[index]].radius;

for(int j=index+1; j<endIndex;j++)
{

  /* Check to see if there is an overlap */
  if( length(PosA-position_com[j]) - (R_A+ParticleObject[ P_ObjectType[j]].radius) < 0.0 )
  {
	  Selected_ID.NN_Index[num_sel+1] = j;
      num_sel++;

  }

}/* End loop thru cell*/

    /* First value is how many selected */
Selected_ID.NN_Index[0] = num_sel;

    return Selected_ID;
}
/*-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
__global__ void Kernel_BroadCollisionDetection_Symmetry(
									  uint   *cellStart,
									  uint   *cellEnd ,
									  uint   *Broad_List,
									  uint   *NN_NUM,
									  float3 *position_com,
									  uint   *P_ObjectID,
									  int    *P_ID,
									  int    Num_Particles  )
{

/* get the unique index */
uint index = blockIdx.x*blockDim.x + threadIdx.x;

if( index < Num_Particles && P_ID[index]>-1)
{

/* read particle data from sorted arrays */
float3 pos_A = position_com[index];


/* get address in grid */
int3 gridPos = SpatialDecomp_CalcGridPos(pos_A);


/* examine neighboring cells */
/* in a forward direction */

int zstart =  0;
int zend   =  1;

int ystart =  0;
int yend   =  1;

int xstart =  0;
int xend   =  1;

/* Snap to corners */
if(gridPos.x==(SimParms.num_NNCells.x-1))
{
xend = 0;
}
if(gridPos.y==(SimParms.num_NNCells.y-1))
{
yend = 0;
}
if(gridPos.z==(SimParms.num_NNCells.z-1))
{
zend = 0;
}

NN_Selected_Cell Selected_Index;


int counter=0;


for (int y = ystart; y <= yend; y++)
{
for (int z = zstart; z <= zend ; z++)
{
    for (int x=  xstart; x <= xend; x++)
    {

      if((x+y+z)>0)/* dont check own cell */
      {
        int3 neighbourPos = gridPos + make_int3(x, y, z);


        Selected_Index = CollisionDetection_Filter_SphereNNCell( neighbourPos, index,cellStart, cellEnd, position_com, P_ObjectID);

        for(int i=0; i <Selected_Index.NN_Index[0];i++)
        {
        	Broad_List[index*32 + (counter+i) ] = Selected_Index.NN_Index[i+1];

        }
        counter = counter + Selected_Index.NN_Index[0];
      }




    }
}
}


/* Need to include the diagonal neighbors */

Selected_Index.NN_Index[0]=0;


if( gridPos.x!=0 && gridPos.z!=(SimParms.num_NNCells.z-1))
{
int3 neighbourPos = gridPos + make_int3(-1, 0, 1);

Selected_Index = CollisionDetection_Filter_SphereNNCell( neighbourPos, index,cellStart, cellEnd,position_com, P_ObjectID );


for(int i=0; i <Selected_Index.NN_Index[0];i++)
{
   Broad_List[index*32 + (counter+i)] = Selected_Index.NN_Index[i+1];

}
counter = counter + Selected_Index.NN_Index[0];

}




Selected_Index.NN_Index[0]=0;
if( gridPos.x!=0 && gridPos.z!=(SimParms.num_NNCells.z-1) && gridPos.y!=(SimParms.num_NNCells.y-1) )
{
int3 neighbourPos = gridPos + make_int3(-1, 1, 1);

Selected_Index = CollisionDetection_Filter_SphereNNCell( neighbourPos, index,cellStart, cellEnd, position_com, P_ObjectID);


for(int i=0; i <Selected_Index.NN_Index[0];i++)
{
   Broad_List[index*32 + (counter+i)] = Selected_Index.NN_Index[i+1];
}
counter = counter + Selected_Index.NN_Index[0];

}


Selected_Index.NN_Index[0]=0;
if( gridPos.x!=(SimParms.num_NNCells.x-1) && gridPos.y!=(SimParms.num_NNCells.y-1) && gridPos.z!=0 )
{
int3 neighbourPos = gridPos + make_int3(1, 1, -1);

Selected_Index = CollisionDetection_Filter_SphereNNCell( neighbourPos, index,cellStart, cellEnd, position_com, P_ObjectID);


for(int i=0; i <Selected_Index.NN_Index[0];i++)
{
   Broad_List[index*32 + (counter+i)] = Selected_Index.NN_Index[i+1];

}
counter = counter + Selected_Index.NN_Index[0];

}



Selected_Index.NN_Index[0]=0;
if( gridPos.x!=0 && gridPos.y!=(SimParms.num_NNCells.y-1) && gridPos.z!=0 )
{
int3 neighbourPos = gridPos + make_int3(-1, 1, -1);

Selected_Index = CollisionDetection_Filter_SphereNNCell( neighbourPos, index,cellStart, cellEnd, position_com, P_ObjectID );


 for(int i=0; i < Selected_Index.NN_Index[0];i++)
{
   Broad_List[index*32 + (counter+i)] = Selected_Index.NN_Index[i+1];
}
counter = counter + Selected_Index.NN_Index[0];

}


Selected_Index.NN_Index[0]=0;
if( gridPos.x!=(SimParms.num_NNCells.x-1) && gridPos.y!=0 )
{
int3 neighbourPos = gridPos + make_int3(1, -1, 0);

Selected_Index = CollisionDetection_Filter_SphereNNCell( neighbourPos, index,cellStart, cellEnd, position_com, P_ObjectID );


 for(int i=0; i <Selected_Index.NN_Index[0];i++)
{
   Broad_List[index*32 + (counter+i)] = Selected_Index.NN_Index[i+1];

}
counter = counter + Selected_Index.NN_Index[0];

}



Selected_Index.NN_Index[0]=0;
if( gridPos.y!=0 && gridPos.z!=(SimParms.num_NNCells.z-1)  )
{
int3 neighbourPos = gridPos + make_int3(0, -1, 1);

Selected_Index = CollisionDetection_Filter_SphereNNCell( neighbourPos, index,cellStart, cellEnd, position_com , P_ObjectID);


 for(int i=0; i < Selected_Index.NN_Index[0];i++)
{
   Broad_List[index*32 + (counter+i)] = Selected_Index.NN_Index[i+1];

}
counter = counter + Selected_Index.NN_Index[0];

}


/*  Own cell Logic */
uint gridHash = SpatialDecomp_CalcGridHash(gridPos);

/* get start of bucket for this cell */
uint startIndex = cellStart[gridHash];
/* Get end of the cell */
uint endIndex = cellEnd[gridHash];


int num_particlesCell = endIndex-startIndex;

/* 2 Particles in cell */
if (num_particlesCell>1 && num_particlesCell<=2)
{
Selected_Index.NN_Index[0]=0;
Selected_Index = CollisionDetection_Filter_2SphereSameCell( gridPos, index,cellStart, cellEnd, position_com, P_ObjectID, P_ID);


for(int i=0; i <Selected_Index.NN_Index[0];i++)
{
	Broad_List[index*32 + (counter+i) ] = Selected_Index.NN_Index[i+1];//make_uint2(index,Selected_Index[i+1]);

}
counter = counter + Selected_Index.NN_Index[0];
}/* get a list of pairs*/
else if (num_particlesCell>2)
{
Selected_Index.NN_Index[0]=0;
        Selected_Index = CollisionDetection_Filter_SphereSameCellMany( gridPos, index,cellStart, cellEnd, position_com, P_ObjectID, P_ID);


        for(int i=0; i <Selected_Index.NN_Index[0];i++)
        {
        	Broad_List[index*32 + (counter+i) ] = Selected_Index.NN_Index[i+1];//make_uint2(index,Selected_Index[i+1]);

        }
        counter = counter + Selected_Index.NN_Index[0];

}



/* Store total number of neighbours */
NN_NUM[index] = counter;
//printf("stored %d  %d \n",(index),counter);


} /* thread check */

/*multi gpu */
/* get the unique index */
//uint index = blockIdx.x*blockDim.x + threadIdx.x + gpu_offset;

}
/*-----------------------------------------------------------------------------*/


__global__ void PrintNN (uint *NumNN, uint *NNList, int *P_ID)
{
//  for ( int i=0; i<SimParms.Num_Particles; i++ )
//  {
//	  //printf( "particle %d has %d NN \n",P_ID[i], NumNN[i]);
//
//	  for ( int j=0; j<NumNN[i]; j++ )
//	  {
//		printf(" %d %d \n",i,NNList[i*32+j]);
//	  }
//  }

printf("\n");

}




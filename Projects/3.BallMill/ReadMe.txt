BallMill simulation in 3D for 90cm mill. 

1. WorldFile "Mill_SPH30 reads" the state the inital config from file. 
   
   To check that everything is working you must get these values after the first rotation: 
   time: 2.72727 Power:   Total = 625.98  Particle: 319.724  Lifter: 156.378 Drum 149.878


2. WorldFile "Mill_SPH30_NoLifter" reads the inital config from file and has no lifters just the mill surface. 

3. WorldFile "Mill_SPH20" reads the inital config from file. 

4. WorldFile "Mill_SPH30_Fill" fills the mill with particles. 

  2.1 Note: Once you are happy with the filling press 'p' 
      and the state will be written as "Start_WorldName" which you can use to start from as in  1-3.
      Alternatively if you specify an end run time, this fill we be created automatically at the end. 

  2.2 Note: COR should be set to <=0.35 for filling and the total particle number must be even. 
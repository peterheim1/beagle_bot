/*

drive dist needs to be in a loop to  reset the values

*/
float Pose(){
  
  double F_L = radians((map (Steer_enc[0],0, 1024, 0, 300))-150);
  double F_R = radians((map (Steer_enc[2],0, 1024, 0, 300))-150);
  double R_L = radians((map (Steer_enc[1],0, 1024, 0, 300))-150);
  double R_R = radians((map (Steer_enc[3],0, 1024, 0, 300))-150);
  drive_F_L = (count_L - Prev_count_F_L);
  drive_F_R = (count_R - Prev_count_F_R);
  drive_R_L = (count_R_L - Prev_count_R_L);
  drive_R_R = (count_R_R - Prev_count_R_R); 
  float Base_A =(F_L + F_R + R_L + R_R)/4;
  float Base_B =(F_L + F_R)/2;
  float Base_Rot =(Base_B - Base_A)/2;
  BaseRotation = ((F_L + F_R)/2) - ((R_L + R_R)/2);
  DriveDist_Front = ((drive_F_L + drive_F_R) * 0.5) * 0.000120;//1 click is meters travled .298/2480
  DriveDist_Rear = ((drive_R_L + drive_R_R) * 0.5) * 0.00008514;//1 click is meters travled     .298/3500
  DriveDist = (DriveDist_Front + DriveDist_Rear) *0.5;
  Hed = sin(DriveDist/(0.2/(sin(Base_Rot))))*1.8;//1.8;//Base_Rot;1.8   ///rotation in radians per second
  
  
  
  Heading += Hed;// needs to be radians traveled in the loop use distance

  float VX = (( DriveDist*cos(F_L+ Heading)) + ( DriveDist*cos(F_R+ Heading)) +( DriveDist*cos(R_L+ Heading))+( DriveDist*cos(R_R+ Heading)))/4;// the orentation of the drive wheels
  float VY = (( DriveDist*sin(F_L+ Heading)) + ( DriveDist*sin(F_R+ Heading)) +( DriveDist*sin(R_L+ Heading))+( DriveDist*sin(R_R+ Heading)))/4;
  Delta_X = VX;//(DriveDist*cos(Base_A));//*.001;//velocity in x direction
  Delta_Y = VY;//(DriveDist*sin(Base_A));//*.001;// velocity in y direction

  //float vel_x = DriveDist_Front;
  //float vel_y = DriveDist_Rear;
  
  //PreviousHeading = DriveAngle;
  X_dist += VX;
  Y_dist += VY;
  Prev_count_F_L = count_L;
  Prev_count_F_R = count_R;
  Prev_count_R_L = count_R_L;
  Prev_count_R_R = count_R_R;
  
  if (Heading > PI)
		{
			Heading -= TwoPI;
		}
		else
		{
			if (Heading <= -PI)
			{
				Heading += TwoPI;
			}
		} 

  //}
  Prev_count_F_L = count_L;
  Prev_count_F_R = count_R;
  Prev_count_R_L = count_R_L;
  Prev_count_R_R = count_R_R;
}




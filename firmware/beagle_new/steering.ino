
//Front right steering pod
void Compute_F_R()
{
    Steer_F_R.Compute();
    Steer_F_R.SetOutputLimits(-90, 90); 



}
 
//left front stearing pod

void Compute_F_L()
{
   Steer_F_L.Compute();
   Steer_F_L.SetOutputLimits(-90, 90); 
}


//right rear steering pod
void Compute_R_R()
{
    Steer_R_R.Compute();
    Steer_R_R.SetOutputLimits(-90, 90); 
}
 
 
//left rear stearing pod

void Compute_R_L()
{
  Steer_R_L.Compute();
  Steer_R_L.SetOutputLimits(-90, 90); 
}






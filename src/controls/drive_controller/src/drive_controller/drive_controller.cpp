#include <drive_controller/drive_controller.h>


DriveController::DriveController(iVescAccess *fr, iVescAccess *fl, iVescAccess *bl, iVescAccess *br)
: front_right_wheel(fr), front_left_wheel (fl), back_left_wheel(bl), back_right_wheel(br),
  p_theta(Gchopsize), p_omega(Gchopsize), p_alpha(Gchopsize), p_lengths(Gchopsize),
  p_x(Gchopsize), p_y(Gchopsize), p_length(0), p_paths(0), p_last_closest_t(0), p_closest_t(0),
  p_speed_cmd(0), p_prev_UlUr(0,0), p_prev_theta(0), p_prev_omega(0)
{


}

void DriveController::addPath(DriveController_ns::bezier_path path)
{
 if (p_paths ==0) //no current paths
 {
    getAngleAndLengthInfo(path, p_theta, p_omega, p_alpha, p_lengths,
                          p_x, p_y, p_length, this->Gchopsize);
    p_paths +=1;
 }
 else return; //one path at a time for now

 return;
}

void DriveController::haltAndAbort()
{
  front_right_wheel->setLinearVelocity(0);
  front_left_wheel->setLinearVelocity(0);
  back_left_wheel->setLinearVelocity(0);
  back_right_wheel->setLinearVelocity(0);

  p_paths = 0;

}


bool DriveController::update(LocalizerInterface::stateVector sv, double dt)
{
  if (p_paths > 0)
  {
  double right_speed = .5*(front_right_wheel->getLinearVelocity() 
                           + back_right_wheel->getLinearVelocity());
  double left_speed  = .5*(front_left_wheel->getLinearVelocity()  
                           + back_left_wheel->getLinearVelocity());
  p_prev_UlUr.first = left_speed;
  p_prev_UlUr.second = right_speed;

  double speed_gain =  1;  /*DNFW*/
  double set_speed  = .4;  /*DNFW*/
  double angle_gain = 10;  /*DNFW*/ 
  double path_gain  = 50;  /*DNFW*/
  

  double steering = 0;
  //limit denominator for steering
    if (std::abs(left_speed + right_speed) < .01)
        {steering =  (2/Axelsize) * (right_speed - left_speed)/.01;}
    else
        {steering = (2/Axelsize) * (right_speed - left_speed)/(left_speed + right_speed);}
  //find closest index and path error
    p_last_closest_t = p_closest_t;
    std::pair<double, double> par_and_err = findCPP2019(sv.x_pos, sv.y_pos, p_x, p_y, p_theta, Gchopsize);
    p_closest_t = par_and_err.first;
    double path_error = par_and_err.second;
    if (p_closest_t < p_last_closest_t)
        {p_closest_t = p_last_closest_t;}

    int index_for_t = p_closest_t*Gchopsize;
    if (index_for_t >= Gchopsize) //Done with path (got orthogonal position to end)
    {
      index_for_t = Gchopsize -1;
      p_paths -= 1;
    }
    else 
    {
      if (index_for_t < 0) 
      {
         index_for_t = 0;
      }

      double angle_error = angleDiff(sv.theta,p_theta.at((int)(index_for_t)));

      p_speed_cmd = p_speed_cmd - speed_gain*(p_speed_cmd - set_speed)*dt;

      //interpolate point to take steering from slightly ahead
      double meters_to_jump = p_speed_cmd * dt;
      double jumped_meters =0;
      int t_jumps = 0;
      /* Skip for now, but does improve accuracy
      while (jumped_meters < meters_to_jump && ((int)(p_closest_t*Gchopsize) + t_jumps) < Gchopsize)
      {
          jumped_meters = jumped_meters + p_lengths.at((int)(p_closest_t*Gchopsize) + t_jumps);
          t_jumps = t_jumps+1;
      } */
      //Get wheel velocities
      std::pair<double, double> UlUr = speedSteeringControl(p_speed_cmd, 
                                       p_theta.at((int)(index_for_t) + t_jumps)
                                       - angle_gain*angle_error - path_gain*path_error,
                                       Axelsize, 2.0);

      front_right_wheel->setLinearVelocity(UlUr.second);
      front_left_wheel->setLinearVelocity(UlUr.first);
      back_left_wheel->setLinearVelocity(UlUr.first);
      back_right_wheel->setLinearVelocity(UlUr.second);

      //Model calculations: what we predict will happen in the next frame.
      //TODO: track disturbance on wheels, 
      //      add estimated disturbance to wheel velocities here.
      double m_dxyth[3];
      firstOrderModel(UlUr, sv.theta, sv.omega, dt, m_dxyth);
      double temp[3];
      firstOrderModel(p_prev_UlUr, p_prev_theta, p_prev_omega, dt, temp);
      double m_ddxyth[3];
      for (int index=0;index<3;index++) 
      {
         m_ddxyth[index] = (m_dxyth[index] - temp[index])/dt;
      }
      delta.x_pos = m_dxyth[0];
      delta.y_pos = m_dxyth[1];
      delta.theta = m_dxyth[2];
      delta.x_vel = m_ddxyth[0];
      delta.y_vel = m_ddxyth[1];
      delta.omega = m_ddxyth[2];
      delta.x_accel = 0;
      delta.y_accel = 0;
      delta.alpha = 0;
      
      p_prev_theta = sv.theta;
      p_prev_omega = sv.omega;
    }

  return true;
  }
  else 
  {
    front_right_wheel->setLinearVelocity(0);
    front_left_wheel->setLinearVelocity(0);
    back_left_wheel->setLinearVelocity(0);
    back_right_wheel->setLinearVelocity(0);
    return false;
  }
}

void DriveController::firstOrderModel(std::pair<double, double> UlUr, double world_theta, double omega_est, double dt, double *xyth)
{

      double m_r_x_rot1;
      double m_r_y_rot1;
      double m_dx;
      double m_dy;
      double m_dth;
      double m_omega = omega_est; //(UlUr.second - UlUr.first)/Axelsize;
      if (std::abs(m_omega) > .01)
      {
        double m_R = Axelsize/2.0 * (UlUr.first + UlUr.second)/(UlUr.second - UlUr.first);
        m_r_x_rot1 =  m_R * std::sin(m_omega * dt);
        m_r_y_rot1 = -m_R * std::cos(m_omega * dt) + m_R;
        m_dx = std::cos(world_theta)*m_r_x_rot1 - std::sin(world_theta)*m_r_y_rot1;
        m_dy = std::sin(world_theta)*m_r_x_rot1 - std::cos(world_theta)*m_r_y_rot1;
        m_dth = ((UlUr.second - UlUr.first)/Axelsize) * dt;
      }
      else
      {
        m_r_x_rot1 = .5*(UlUr.first + UlUr.second);
        m_r_y_rot1 = 0;
        m_dx = std::cos(world_theta)*m_r_x_rot1 - std::sin(world_theta)*m_r_y_rot1;
        m_dy = std::sin(world_theta)*m_r_x_rot1 - std::cos(world_theta)*m_r_y_rot1;
        m_dth = 0;
      }
     xyth[0] = m_dx;
     xyth[1] = m_dy;
     xyth[2] = m_dth;
}

LocalizerInterface::stateVector DriveController::getDeltaStateVector()
{
  return delta;
}

double DriveController::getPClosestT()
{
  return p_closest_t;
}

int DriveController::getPPaths()
{
  return p_paths;
}

double DriveController::angleDiff(double angle1, double angle2) //TODO Unit test
{
  double diff = std::atan2(std::sin(angle1 - angle2), std::cos(angle1 - angle2));
  return diff;
}

std::pair<double, double> DriveController::speedSteeringControl(double speed,   double steering, 
                                               double AxelLen, double MaxSpeed)
{
  double maxabs;
  double varargin_2;
  std::pair<double, double> UlUr;
  UlUr.first = 0;
  UlUr.second = 0;

  /* if speed is negative */
  /* positve radius is backwards and to the right (same circle as positive */
  /* speed) */
  /* negative radius is backwards and to the left (same circle as positive */
  /* speed) */
  /* max speed is absolute value */
  /* sign of speed indicates direction */
  /* Speed is .5*(LeftAngularVel + RightAngularVel)*WheelRadius; (angularVel in Rad) */
  /* Turn Radius is DistanceBetweenWheels/2 * (LeftAngularVel + RightAngularVel)/(RightAngularVel - LeftAngularVel) */
  if (std::abs(steering) < 1000.0 && std::abs(steering) > .01) {
    UlUr.first = (4.0 * (1.0 / steering) * speed / AxelLen - 2.0 * speed) * AxelLen /
      ((1.0 / steering) * 4.0);
    UlUr.second = 2.0 * speed - UlUr.first;

    maxabs = std::abs(UlUr.first);
    varargin_2 = std::abs(UlUr.second);
    maxabs = (maxabs > varargin_2) ? maxabs : varargin_2;

    if (maxabs > MaxSpeed) {
      speed = speed * MaxSpeed / maxabs;
      UlUr.first = (4.0 * (1.0 / steering) * speed / AxelLen - 2.0 * speed) * AxelLen /
        ((1.0 / steering) * 4.0);
      UlUr.second = 2.0 * speed - UlUr.first;
    }
  } else if  (std::abs(steering) >= 1000.0)
  {
    UlUr.first = -0.5 * speed;
    UlUr.second = 0.5 * speed;
  }
   else if (std::abs(steering) <=.01)
  {
    UlUr.first =  speed;
    UlUr.second = speed;
  }

   return UlUr;
}

bool DriveController::getAngleAndLengthInfo(DriveController_ns::bezier_path path, 
                           std::vector<double>  &theta, std::vector<double>  &omega, 
                           std::vector<double>  &alpha, std::vector<double>  &lengths, 
                           std::vector<double>  &x, std::vector<double>  &y, double &length, 
                           int chopsize) 
{
    //int chopsize = 100;
    if (theta.size() < chopsize ||
        alpha.size() < chopsize ||
        omega.size() < chopsize ||
        lengths.size() < chopsize ||
        x.size() < chopsize ||
        y.size() < chopsize) return false;

    double t = 0;
    double dt = 1.0/chopsize;
    for (int index =0; index<chopsize; index++)
    {
        double xd1 = 3*(1-t)*(1-t) * (path.x2 - path.x1) + 6*(1-t)*t*(path.x3 - path.x2) + 3*t*t*(path.x4 - path.x3);
        double yd1 = 3*(1-t)*(1-t) * (path.y2 - path.y1) + 6*(1-t)*t*(path.y3 - path.y2) + 3*t*t*(path.y4 - path.y3);
        //we already have all the d1's, might as well do length here
        //since we will need it
        x.at(index) = (1-t)*(1-t)*(1-t) * path.x1 + 3*(1-t)*(1-t) *t*path.x2 + 3*(1-t)*t*t *path.x3 + t*t*t * path.x4;
        y.at(index) = (1-t)*(1-t)*(1-t) * path.y1 + 3*(1-t)*(1-t) *t*path.y2 + 3*(1-t)*t*t *path.y3 + t*t*t * path.y4;
        theta.at(index) = atan2(yd1, xd1);

        t += dt;
    }
    length =0;
    for (int index = 0; index < (chopsize-2); index++)
    {
        lengths.at(index) = std::sqrt((x.at(index+1) - x.at(index))*(x.at(index+1) - x.at(index))
                            + (y.at(index+1) - y.at(index))*(y.at(index+1) - y.at(index)));
        length+=lengths.at(index);
    }
    
    for (int index = 0;index <(chopsize-2);index++)
    {
        omega.at(index) = (theta.at(index+1) - theta.at(index))*chopsize/length; 
        //normalized by path length, 
        //TODO: should probably use estimate from at that point
    }  
    //repeat final values
    omega.at(chopsize-1) = omega.at(chopsize-2);
    
    for (int index = 0; index <(chopsize-3);index++)
    { 
        alpha.at(index) = (omega.at(index+1) - omega.at(index))*chopsize/length;
    }
    //repeat final values, TODO:what happens if we shift?
    alpha.at(chopsize-2) = alpha.at( chopsize-3);
    alpha.at(chopsize-1) = alpha.at(chopsize-2);


    return true;
}

std::pair<double, double> DriveController::findCPP2019(double rx, double ry, 
                                                       std::vector<double>  &x, 
                                                       std::vector<double>  &y,
                                                       std::vector<double> &theta, int chopsize)
{
  double curr_dist;
  double smallest_dist = 1000000;
  int smallest_index = chopsize -1;
  for (int index=0; index < chopsize; index++) //This could be algebra instead
  {
    curr_dist = (x.at(index) -rx)*(x.at(index) -rx) + (y.at(index) - ry)*(y.at(index) - ry);
    if (curr_dist < smallest_dist)
    {
       smallest_dist = curr_dist;
       smallest_index = index;
    }
  }

  double pos_and_cpp_angle = std::atan2(ry - y.at(smallest_index), rx - x.at(smallest_index));
  double error_angle_for_sign = DriveController::angleDiff(pos_and_cpp_angle, theta.at(smallest_index));
  std::pair<double, double> par_and_err;
  par_and_err.first =((double)(smallest_index+1)) /((double) (chopsize));
  par_and_err.second = (error_angle_for_sign > 0 ? 1.0 : -1.0)*std::sqrt(smallest_dist);

  return par_and_err;
}


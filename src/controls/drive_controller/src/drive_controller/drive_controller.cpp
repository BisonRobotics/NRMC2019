#include <drive_controller/drive_controller.h>


DriveController::DriveController(iVescAccess *fr, iVescAccess *fl, iVescAccess *bl, iVescAccess *br)
: front_left_wheel(fl), front_right_wheel (fr), back_left_wheel(bl), back_right_wheel(br),
  p_theta(Gchopsize), p_omega(Gchopsize), p_alpha(Gchopsize), p_lengths(Gchopsize),
  p_x(Gchopsize), p_y(Gchopsize), p_length(0), p_paths(0), p_last_closest_t(0), p_closest_t(0)
{


}

void DriveController::addPath(DriveController_ns::bezier_path path)
{
 if (p_paths ==0) //no current paths
 {
    getAngleAndLengthInfo(path, p_theta, p_omega, p_alpha, p_lengths,
                          p_x, p_y, p_length, this->Gchopsize);
 }
 else return; //one path at a time for now

 return;
}

void DriveController::haltAndAbort()
{


}


bool DriveController::update(DriveController_ns::robot_state_vector sv, double dt)
{
  //double 


  return false;
}

double DriveController::angleDiff(double angle1, double angle2) //TODO Unit test
{
  double diff = std::atan2(std::sin(angle1 - angle2), std::cos(angle1 - angle2));
  return diff;
}

//TODO test
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
  if (std::abs(steering) < 1000.0) {
    UlUr.first = (4.0 * (1.0 / steering) * speed / AxelLen - 2.0 * speed) * AxelLen /
      (1.0 / steering * 4.0);
    UlUr.second = 2.0 * speed - UlUr.first;

    maxabs = std::abs(UlUr.first);
    varargin_2 = std::abs(UlUr.second);
    maxabs = (maxabs > varargin_2) ? maxabs : varargin_2;

    if (maxabs > MaxSpeed) {
      speed = speed * MaxSpeed / maxabs;
      UlUr.first = (4.0 * (1.0 / steering) * speed / AxelLen - 2.0 * speed) * AxelLen /
        (1.0 / steering * 4.0);
      UlUr.second = 2.0 * speed - UlUr.first;
    }
  } else {
    UlUr.first = -0.5 * speed;
    UlUr.second = 0.5 * speed;
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
  for (int index=0; index < chopsize; index++)
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
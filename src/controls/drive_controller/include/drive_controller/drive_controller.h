#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

#include <vector>
#include <utility>

namespace DriveController_ns
{
//TODO add robot_state vector as data type
typedef struct path_s
{
   double x1;
   double y1;
   double x2;
   double y2;
   double x3;
   double y3;
   double x4;
   double y4;
} bezier_path;
typedef struct robot_state_s
{
   double x;
   double y;
   double theta;
   /*double dx;
   double dy;
   double omega;
   double dx2;
   double dy2;*/
   //double l_wheel; //gets these from the VESC layer
   //double r_wheel;
} robot_state_vector;
}

class DriveController
{
public:
   DriveController(iVescAccess *fr, iVescAccess *fl, iVescAccess *bl, iVescAccess *br);
   void addPath(DriveController_ns::bezier_path path);
   void haltAndAbort();
   bool update(DriveController_ns::robot_state_vector sv, double dt);
   double getPClosestT();
   int getPPaths();
//protected:
   double angleDiff(double angle1,double angle2);
   std::pair<double, double> speedSteeringControl(double speed, double steering, 
                                                  double axelLen, double maxSpeed);
   bool getAngleAndLengthInfo(DriveController_ns::bezier_path path, 
                           std::vector<double>  &theta, std::vector<double>  &omega, 
                           std::vector<double>  &alpha, std::vector<double>  &lengths, 
                           std::vector<double>  &x, std::vector<double>  &y, double &length,
                           int chopsize);
   //findCPP2019 finds the closest point on the path described by x and y vectors and 
   // returns the parameter value in [0,1] followed by the signed path error as a pair.
   std::pair<double, double> findCPP2019(double rx, double ry, 
                                         std::vector<double>  &x, 
                                         std::vector<double>  &y,
                                         std::vector<double> &theta, int chopsize);
   
private:
  static const int Gchopsize = 100;
  static constexpr double Axelsize = .5;
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
  std::vector<double>  p_theta;
  std::vector<double>  p_omega;
  std::vector<double>  p_alpha;
  std::vector<double>  p_lengths;
  std::vector<double>  p_x;
  std::vector<double>  p_y;
  double p_length;
  int p_paths;

  double p_last_closest_t;
  double p_closest_t;

  double p_speed_cmd;
};
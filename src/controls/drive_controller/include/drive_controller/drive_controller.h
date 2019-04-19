#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>
#include <localizer/localizer_interface.h>

#include <vector>
#include <utility>

namespace DriveController_ns
{
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
typedef struct error_states_s
{
    double path_error;
    double angle_error;
} error_state;
typedef struct wheel_states_s
{
    double left_wheel_planned;
    double right_wheel_planned;
    double left_wheel_actual;
    double right_wheel_actual;
    double left_wheel_command;
    double right_wheel_command;
} wheel_state;
typedef struct path_info_s
{
    double path_theta;
    double path_omega;
} path_info;
}

class DriveController
{
public:
   DriveController(iVescAccess *fr, iVescAccess *fl, iVescAccess *bl, iVescAccess *br);
   void addPath(DriveController_ns::bezier_path path, bool forward_point);
   void haltAndAbort();
   bool update(LocalizerInterface::stateVector sv, double dt);
   LocalizerInterface::stateVector getDeltaStateVector();
   double getPClosestT();
   int getPPaths();
   DriveController_ns::error_state getErrorStates();
   DriveController_ns::wheel_state getWheelStates();
   DriveController_ns::path_info getPathInfo();
   void setMaxSpeed(double speed);
//protected:
   double angleDiff(double angle1,double angle2);
   std::pair<double, double> speedSteeringControl(double speed, double steering, 
                                                  double axelLen, double maxSpeed);
   bool getAngleAndLengthInfo(DriveController_ns::bezier_path path, 
                           std::vector<double>  &theta, std::vector<double>  &omega, 
                           std::vector<double>  &alpha, std::vector<double>  &lengths, 
                           std::vector<double>  &x, std::vector<double>  &y, double &length,
                           int chopsize, bool forward_path);
   //findCPP2019 finds the closest point on the path described by x and y vectors and 
   // returns the parameter value in [0,1] followed by the signed path error as a pair.
   std::pair<double, double> findCPP2019(double rx, double ry, 
                                         std::vector<double>  &x, 
                                         std::vector<double>  &y,
                                         std::vector<double> &theta, int chopsize);
   void firstOrderModel(std::pair<double, double> UlUr, double world_theta, double omega_est, double dt, double *xyth);
   bool cleanPath(DriveController_ns::bezier_path *path, double x, double y, double theta, bool fwd);
   
private:
  static const int Gchopsize = 100;
  static constexpr double Axelsize = 1.0;
  double max_speed;
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
  std::vector<double>  p_theta;
  std::vector<double>  p_omega;
  std::vector<double>  p_alpha;
  std::vector<double>  p_lengths;
  std::vector<double>  p_x;
  std::vector<double>  p_y;
  double p_length;
  int p_paths;
  bool p_forward_point;

  double p_last_closest_t;
  double p_closest_t;

  double p_speed_cmd;
  double p_steering_cmd;

  LocalizerInterface::stateVector delta;
  std::pair<double,double> p_prev_UlUr;
  double p_prev_theta;
  double p_prev_omega;
  DriveController_ns::error_state es;
  DriveController_ns::wheel_state ws;
};
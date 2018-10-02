#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

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
}bezier_path;
}

class DriveController
{
public:
   DriveController(iVescAccess *fr, iVescAccess *fl, iVescAccess *bl, iVescAccess *br);
   //void addPath(DriveController_ns::bezier_path path);
   //void haltAndAbort();
   //bool update(double robotX, double robotY, double robotTheta);
//protected:
   double angleDiff(double angle1,double angle2);
   std::pair<double, double> speedSteeringControl(double speed, double steering, 
                                                  double axelLen, double maxSpeed);
bool getAngleAndLengthInfo(DriveController_ns::bezier_path path, 
                           std::vector<double>  &theta, std::vector<double>  &omega, 
                           std::vector<double>  &alpha, std::vector<double>  &lengths, 
                           std::vector<double>  &x, std::vector<double>  &y, double &length,
                           int chopsize);
   
private:
  static const int Gchopsize = 100;
  iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
  std::vector<double> lengths;
  std::vector<double> angles;
};
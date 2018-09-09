#include <vesc_access/vesc_access.h>
#include <vesc_access/ivesc_access.h>

#include <vector.h>
#include <pair.h>

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
   void addPath(DriveController_ns::bezier_path path);
   void haltAndAbort();
   bool update(double robotX, double robotY, double robotTheta);
//protected:
   double angleDiff(angle1, angle2);
   std::pair<double, double> speedSteeringControl(double speed, double steering, 
                                                  double axelLen, double maxSpeed);
   std::pair<std::vector<double>,std::vector<double> >
         getAngleAndLengthInfo(DriveController_ns::bezier_path path);
   
private:
   static int chopsize = 100;
   iVescAccess *front_left_wheel, *front_right_wheel, *back_right_wheel, *back_left_wheel;
};
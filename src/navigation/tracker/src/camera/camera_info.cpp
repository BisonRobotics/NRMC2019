#include <tracker/camera/camera_info.h>

tracker::CameraInfo::CameraInfo(uint width, uint height, double fx,
                                double fy, double cx, double cy):
    width(width), height(height), fx(fx), fy(fy), cx(cx), cy(cy) {}


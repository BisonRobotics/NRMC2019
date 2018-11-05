#ifndef DRIVER_ACCESS_DIRECTION_H
#define DRIVER_ACCESS_DIRECTION_H

namespace navigation_msgs
{
enum class Direction : int8_t
{
    none = 0,
    forward = 1,
    reverse = -1
};
}

#endif //DRIVER_ACCESS_DIRECTION_H

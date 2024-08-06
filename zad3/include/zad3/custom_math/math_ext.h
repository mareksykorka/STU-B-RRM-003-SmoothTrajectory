#ifndef CATKIN_TRETIE_ZADANIE_MATH_EXT_H
#define CATKIN_TRETIE_ZADANIE_MATH_EXT_H

#include <ros/ros.h>

double degToRad(double degrees)
{
    return degrees * M_PI / 180.0;
}

double radToDeg(double radians)
{
    return radians * 180.0 / M_PI;
}

#endif //CATKIN_TRETIE_ZADANIE_MATH_EXT_H
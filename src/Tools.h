#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <vector>

#include "Coordinates.h"

class Tools
{
public:
    
    static double deg2rad(const double x);
    
    static double rad2deg(const double x);
    
    static void splitWorldCoordinates(std::vector<double>& x_vector, std::vector<double>& y_vector, const std::vector<WorldCoordinates>& input);
};

#endif //_TRAFFIC_H_

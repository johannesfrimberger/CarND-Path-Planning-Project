#ifndef _TOOLS_H_
#define _TOOLS_H_

#include <vector>

#include "Coordinates.h"

class Tools
{
public:
    
    /** Convert angle in degree to radians */
    static double deg2rad(const double x);
    
    /** Convert angle in radians to degree */
    static double rad2deg(const double x);
    
    /** Convert velocity in meter per second to miles per hour */
    static double mps2mph(const double x);
    
    /** Convert velocity in miles per hour to meter per second */
    static double mph2mps(const double x);
    
    /** Split vector in WorldCoordinates to a vector of separate x and y values */
    static void splitWorldCoordinates(std::vector<double>& x_vector, std::vector<double>& y_vector, const std::vector<WorldCoordinates>& input);
private:
    
    // Conversion factor for meter per second to miles per hour
    static const double C_CONVERT_MPS_2_MPH;
};

#endif //_TRAFFIC_H_

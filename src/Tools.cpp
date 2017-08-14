#include "Tools.h"

#include <math.h>

const double Tools::C_CONVERT_MPS_2_MPH = 2.24;

double Tools::NormalizeAngle(const double angle)
{
    double result = angle;
    
    while (result > M_PI)
    {
        result -= 2. * M_PI;
    }
    
    while (result < -M_PI)
    {
        result += 2.*M_PI;
    }
    
    return result;
}

double Tools::deg2rad(const double x)
{
    return NormalizeAngle(x * M_PI / 180.0);
}

double Tools::rad2deg(const double x)
{
    return NormalizeAngle(x) * 180.0 / M_PI;
}

void Tools::splitWorldCoordinates(std::vector<double>& x_vector, std::vector<double>& y_vector, const std::vector<WorldCoordinates>& input)
{
    x_vector.clear();
    y_vector.clear();
    
    for(auto el : input)
    {
        x_vector.push_back(el.getX());
        y_vector.push_back(el.getY());
    }
}

double Tools::mps2mph(const double x)
{
    return x * C_CONVERT_MPS_2_MPH;
}

double Tools::mph2mps(const double x)
{
    return x / C_CONVERT_MPS_2_MPH;
}

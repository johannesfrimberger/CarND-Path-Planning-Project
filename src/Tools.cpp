#include "Tools.h"

#include <math.h>

double Tools::deg2rad(const double x)
{
    return x * M_PI / 180.0;
}

double Tools::rad2deg(const double x)
{
    return x * 180.0 / M_PI;
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
    return x * 2.24;
}

double Tools::mph2mps(const double x)
{
    return x / 2.24;
}

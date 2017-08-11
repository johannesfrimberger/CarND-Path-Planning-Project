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

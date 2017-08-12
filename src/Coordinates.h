#ifndef _COORDINATES_H_
#define _COORDINATES_H_

#include <math.h>

/**  */
class WorldCoordinates
{
public:
    
    explicit WorldCoordinates(const double x_in, const double y_in):
        x(x_in),
        y(y_in)
    { }
    
    // Overload + operator
    WorldCoordinates operator+(const WorldCoordinates& b) const
    {
        return WorldCoordinates(this->x + b.x, this->y + b.y);
    }
    
    // Overload - operator
    WorldCoordinates operator-(const WorldCoordinates& b) const
    {
        return WorldCoordinates(this->x - b.x, this->y - b.y);
    }
    
    double getX() const
    {
        return x;
    }
    
    double getY() const
    {
        return y;
    }
    
    double getDistance(const WorldCoordinates ref) const
    {
        return sqrt(pow(ref.x - x, 2) + pow(ref.y - y, 2));
    }
    
    double getHeading(const WorldCoordinates ref) const
    {
        return atan2((ref.y - y), (ref.x - x));
    }
    
    WorldCoordinates getProjection(const WorldCoordinates ref) const
    {
        const double proj_norm = (ref.x * x + ref.y * y) / (x * x + y * y);
        return WorldCoordinates(proj_norm * x, proj_norm * y);
    }
    
private:
    
    double x;
    double y;
    
};

/**  */
class FrenetCoordinates
{
public:
    
    explicit FrenetCoordinates(const double s_in, const double d_in):
    s(s_in),
    d(d_in)
    { }

    double getS() const
    {
        return s;
    }
    
    double getD() const
    {
        return d;
    }
    
private:
    
    double s;
    double d;
    
};

#endif //_COORDINATES_H_

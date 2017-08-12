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
    
    // Overload * operator
    WorldCoordinates operator*(const WorldCoordinates& b) const
    {
        return WorldCoordinates(this->x * b.x, this->y * b.y);
    }
    
    // Overload * operator
    WorldCoordinates operator*(const double& b) const
    {
        return WorldCoordinates(this->x * b, this->y * b);
    }
    
    double getX() const
    {
        return x;
    }
    
    double getY() const
    {
        return y;
    }
    
    double getDistance(const WorldCoordinates& ref) const
    {
        return sqrt(pow(ref.x - x, 2) + pow(ref.y - y, 2));
    }
    
    double getLength() const
    {
        return  sqrt(pow(x, 2) + pow(y, 2));
    }
    
    double getHeading(const WorldCoordinates& ref) const
    {
        return atan2((ref.y - y), (ref.x - x));
    }
    
    WorldCoordinates getProjection(const WorldCoordinates& ref) const
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
    
    // Overload + operator
    FrenetCoordinates operator+(const FrenetCoordinates& b) const
    {
        return FrenetCoordinates(this->s + b.s, this->d + b.d);
    }
    
    // Overload - operator
    FrenetCoordinates operator-(const FrenetCoordinates& b) const
    {
        return FrenetCoordinates(this->s - b.s, this->d - b.d);
    }
    
    // Overload * operator
    FrenetCoordinates operator*(const FrenetCoordinates& b) const
    {
        return FrenetCoordinates(this->s * b.s, this->d * b.d);
    }
    
    // Overload * operator
    FrenetCoordinates operator*(const double& b) const
    {
        return FrenetCoordinates(this->s * b, this->d * b);
    }

    double getS() const
    {
        return s;
    }
    
    double getD() const
    {
        return d;
    }
    
    double getDistance(const FrenetCoordinates& ref) const
    {
        return (s - ref.s);
    }
    
    unsigned getLane() const
    {
        return static_cast<unsigned>(d / 4.0);
    }
    
private:
    
    double s;
    double d;
    
};

#endif //_COORDINATES_H_

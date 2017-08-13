#ifndef _COORDINATES_H_
#define _COORDINATES_H_

#include <math.h>

/** Class to keep X and Y positions */
class WorldCoordinates
{
public:
    
    /** Explicit constructor */
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
    
    /** Get x position */
    double getX() const
    {
        return x;
    }
    
    /** Get y position */
    double getY() const
    {
        return y;
    }
    
    /** Get distance to reference position */
    double getDistance(const WorldCoordinates& ref) const
    {
        return sqrt(pow(ref.x - x, 2) + pow(ref.y - y, 2));
    }
    
    /** Get heading relative to reference position */
    double getHeading(const WorldCoordinates& ref) const
    {
        return atan2((ref.y - y), (ref.x - x));
    }
    
    /** Get length of position vector */
    double getLength() const
    {
        return  sqrt(pow(x, 2) + pow(y, 2));
    }
    
    /** Get prejection of position to reference position */
    WorldCoordinates getProjection(const WorldCoordinates& ref) const
    {
        const double proj_norm = (ref.x * x + ref.y * y) / (x * x + y * y);
        return WorldCoordinates(proj_norm * x, proj_norm * y);
    }
    
private:
    
    // Internal storage of position x
    double x;
    
    // Internal storage of position y
    double y;
    
};

/** Class to store Frenet coordinates */
class FrenetCoordinates
{
public:
    
    /** Explicit constructor */
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

    /** Get frenet s position */
    double getS() const
    {
        return s;
    }
    
    /** Get frenet d position */
    double getD() const
    {
        return d;
    }
    
    /** Get distance to reference object */
    double getDistance(const FrenetCoordinates& ref) const
    {
        return (s - ref.s);
    }
    
    /** Convert d position to lane (asuming lane-width of 4m) */
    unsigned getLane() const
    {
        return static_cast<unsigned>(d / 4.0);
    }
    
private:
    
    // Internal storage of s position
    double s;
    
    // Internal storage of d positiion
    double d;
    
};

#endif //_COORDINATES_H_

#ifndef _MAP_H_
#define _MAP_H_

#include <iostream>
#include <vector>

#include "Coordinates.h"

class Waypoint
{
public:
    
    Waypoint(double x, double y, float s, float d_x, float d_y);
    
    const WorldCoordinates& getWorldCoordinats() const
    {
        return world;
    }
    
    const FrenetCoordinates& getFrenet() const
    {
        return frenet;
    }
    
private:
    
    const WorldCoordinates world; //
    const WorldCoordinates world_normal; //
    const FrenetCoordinates frenet; //
};

/**  */
class Map
{
public:
    
    Map(std::string filename);
    
    FrenetCoordinates getFrenet(const WorldCoordinates& pos, const double theta) const;
    
    WorldCoordinates getXY(const FrenetCoordinates& pos) const;
    
private:
    
    // The max s value before wrapping around the track back to 0
    const double max_s = 6945.554;
    
    unsigned getClosestWaypoint(const WorldCoordinates& pos) const;
    
    unsigned getNextWaypoint(const WorldCoordinates& pos, const double theta) const;
    
    std::vector<Waypoint> waypoints; // Storage of all map waypoints
    
};

#endif //_MAP_H_

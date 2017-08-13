#ifndef _MAP_H_
#define _MAP_H_

#include <iostream>
#include <vector>

#include "Coordinates.h"

/**  */
class Map
{
public:
    
    /** Explicit constructor */
    explicit Map(std::string filename);
    
    /** Convert current position to frenet coordinats */
    FrenetCoordinates getFrenet(const WorldCoordinates& pos, const double theta) const;
    
    /** Convert frenet coordinats to x,y position */
    WorldCoordinates getXY(const FrenetCoordinates& pos) const;
    
private:
    
    // The max s value before wrapping around the track back to 0
    //const double max_s = 6945.554;
    
    /** Data structure to hold waypoints */
    class Waypoint
    {
    public:
        
        /** Explicit constructor */
        explicit Waypoint(double x, double y, float s, float d_x, float d_y):
        world(x,y),
        world_normal(d_x, d_y),
        frenet(s, 0)
        {
        }
        
        /** Access to world coordinates */
        const WorldCoordinates& getWorldCoordinats() const
        {
            return world;
        }
        
        /** Access to frenet coordinates */
        const FrenetCoordinates& getFrenet() const
        {
            return frenet;
        }
        
    private:
        
        // Internal storage of world coordinates
        const WorldCoordinates world;
        
        // Internal storage of normal to world coordinates
        const WorldCoordinates world_normal;
        
        // Internal storage of frenet coordinates
        const FrenetCoordinates frenet;
    };
    
    /** Get closest waypoint to world coordinates */
    unsigned getClosestWaypoint(const WorldCoordinates& pos) const;
    
    /** Get next waypoint to world coordinates */
    unsigned getNextWaypoint(const WorldCoordinates& pos, const double theta) const;
    
    // Internal storage of all waypoints on map
    std::vector<Waypoint> waypoints;
    
};

#endif //_MAP_H_

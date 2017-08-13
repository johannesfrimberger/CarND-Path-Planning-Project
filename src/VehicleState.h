#ifndef _VEHICLE_STATE_H_
#define _VEHICLE_STATE_H_

#include "json.hpp"
#include <vector>

#include "Coordinates.h"

using json = nlohmann::json;

/** Class to read and store current vehicle state */
class VehicleState
{
public:
    
    /** Default constructor */
    VehicleState();
    
    /** Reset all internal states */
    void reset();
    
    /** Update internal state from sensor readings */
    void update(const json input);
    
    /** Get number of elements in previous path */
    unsigned getNumberOfPreviousPathElements() const
    {
        return previous_path.size();
    }
    
    /** Get all elements in previous path */
    const std::vector<WorldCoordinates>& getPreviousPath() const
    {
        return previous_path;
    }
    
    /** Get previous path element at position i */
    const WorldCoordinates& getPreviousPathElement(const unsigned i) const
    {
        return previous_path[i];
    }
    
    /** Get previous path element at end position - i */
    const WorldCoordinates& getPreviousPathElementReverse(const unsigned i) const
    {
        return previous_path[getNumberOfPreviousPathElements() - i - 1];
    }
    
    /** Get current vehicle position in world coordinats */
    const WorldCoordinates& getWorldCoordinats() const
    {
        return world;
    }
    
    /** Get current vehicle position in frenet coordinats */
    const FrenetCoordinates& getFrenet() const
    {
        return frenet;
    }
    
    /** Get current vehicle speed (m/s) */
    const double getSpeed() const
    {
        return speed;
    }
    
    /** Get current vehicle yaw-rate (rad/s) */
    const double getYaw() const
    {
        return yaw;
    }
    
    /** Simulate vehicle position in dt seconds */
    VehicleState simulate(const double dt) const;
    
private:
    
    // Internal storage of previous path
    std::vector<WorldCoordinates> previous_path;
    
    // Internal storage of world coordinats
    WorldCoordinates world;
    
    // Internal storage of frenet coordinats
    FrenetCoordinates frenet;
    
    // Internal storage of speed (m/s)
    double speed;
    
    // Internal storage of yaw-rate (rad/s)
    double yaw;
    
    // End of stored path in frenet coordinats
    FrenetCoordinates end_path;
};

#endif //_VEHICLE_STATE_H_

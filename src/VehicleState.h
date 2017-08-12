#ifndef _VEHICLE_STATE_H_
#define _VEHICLE_STATE_H_

#include "json.hpp"
#include <vector>

#include "Coordinates.h"

using json = nlohmann::json;

class VehicleState
{
public:
    
    VehicleState();
    
    void update(const json input);
    
    unsigned getNumberOfPreviousPathElements() const
    {
        return previous_path.size();
    }
    
    const std::vector<WorldCoordinates>& getPreviousPath() const
    {
        return previous_path;
    }
    
    const WorldCoordinates& getPreviousPathElement(const unsigned i) const
    {
        return previous_path[i];
    }
    
    const WorldCoordinates& getPreviousPathElementReverse(const unsigned i) const
    {
        return previous_path[getNumberOfPreviousPathElements() - i - 1];
    }
    
    const WorldCoordinates& getWorldCoordinats() const
    {
        return world;
    }
    
    const FrenetCoordinates& getFrenet() const
    {
        return frenet;
    }
    
    const double getSpeed() const
    {
        return speed;
    }
    
    const double getYaw() const
    {
        return yaw;
    }
    
private:
    
    std::vector<WorldCoordinates> previous_path;
    WorldCoordinates world;
    FrenetCoordinates frenet;
    double speed;
    double yaw;
    double end_path_s;
    double end_path_d;
    
};

#endif //_VEHICLE_STATE_H_

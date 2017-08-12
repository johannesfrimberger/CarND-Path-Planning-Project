#ifndef _TRAFFIC_H_
#define _TRAFFIC_H_

#include "json.hpp"

#include "Coordinates.h"
#include "VehicleState.h"

// for convenience
using json = nlohmann::json;

class VehicleState;

/**  */
class Traffic
{
public:
    
    Traffic(const json input);
    
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
        return speed.getLength();
    }
    
    void addToRelevantList(std::vector<Traffic>& front, std::vector<Traffic>& rear, const VehicleState& ego) const;
    
    Traffic simulate(const double dt) const;
    
private:
    
    WorldCoordinates world;
    WorldCoordinates speed;
    FrenetCoordinates frenet;
    const unsigned id;
    
};

#endif //_TRAFFIC_H_

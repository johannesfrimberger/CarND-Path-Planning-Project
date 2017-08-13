#ifndef _TRAFFIC_H_
#define _TRAFFIC_H_

#include "json.hpp"

#include "Coordinates.h"
#include "VehicleState.h"

// for convenience
using json = nlohmann::json;

class VehicleState;
class RelativTraffic;

/**  */
class Traffic
{
public:
    
    Traffic(const json input);
    
    const unsigned getId() const
    {
        return traffic_id;
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
        return speed.getLength();
    }
    
    void addToRelevantList(std::vector<RelativTraffic>& front, std::vector<RelativTraffic>& rear, const VehicleState& ego) const;
    
    Traffic simulate(const double dt) const;
    
protected:
    
    WorldCoordinates world;
    WorldCoordinates speed;
    FrenetCoordinates frenet;
    const unsigned traffic_id;
    
};

class RelativTraffic : public Traffic
{
public:
    RelativTraffic(const Traffic traffic, const VehicleState& ego);
    
    double getDistance() const
    {
        return distance;
    }
    
    bool isVehicleInFront() const
    {
        return in_front;
    }
    
private:
    
    double distance;
    bool in_front;
};


#endif //_TRAFFIC_H_

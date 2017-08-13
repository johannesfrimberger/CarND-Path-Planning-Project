#ifndef _TRAFFIC_H_
#define _TRAFFIC_H_

#include "json.hpp"

#include "Coordinates.h"
#include "VehicleState.h"

// for convenience
using json = nlohmann::json;

// Forward declarations
class VehicleState;
class RelativTraffic;

/** Traffic object */
class Traffic
{
public:
    
    /** Explicit Constructor */
    Traffic(const json input);
    
    /** Get id of traffic object */
    const unsigned getId() const
    {
        return traffic_id;
    }
    
    /** Get position in world coordinates */
    const WorldCoordinates& getWorldCoordinats() const
    {
        return world;
    }
    
    /** Get position in frenet coordinates */
    const FrenetCoordinates& getFrenet() const
    {
        return frenet;
    }
    
    /** Get speed of the vehicle */
    const double getSpeed() const
    {
        return speed.getLength();
    }
    
    /** Convert Traffic object to RelativTraffic and push it to front or rear list depending on its position */
    void addToRelevantList(std::vector<RelativTraffic>& front, std::vector<RelativTraffic>& rear, const VehicleState& ego) const;
    
    /** Simulate constant movement for dt seconds */
    Traffic simulate(const double dt) const;
    
protected:
    
    // Internal storage of world coordinates
    WorldCoordinates world;
    
    // Internal storage of speed relative to world coordinates
    WorldCoordinates speed;
    
    // Internal storage of frenet coordinates
    FrenetCoordinates frenet;
    
    // Internal storage of id
    const unsigned traffic_id;
};

/** Traffic object with additional information relativ to another vehicle */
class RelativTraffic : public Traffic
{
public:
    
    /** Explicit Constructor */
    explicit RelativTraffic(const Traffic traffic, const VehicleState& ego);
    
    /** Get absolute distance to reference object */
    double getDistance() const
    {
        return distance;
    }
    
    /** Check if traffic is in front of reference object */
    bool isVehicleInFront() const
    {
        return in_front;
    }
    
private:
    
    // Distance to reference object
    double distance;
    
    // Traffic object is in front of reference object
    bool in_front;
};


#endif //_TRAFFIC_H_

#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "json.hpp"
#include <vector>

#include "Map.h"
#include "Traffic.h"
#include "VehicleState.h"

using json = nlohmann::json;

/**  */
class Vehicle
{
public:
    
    Vehicle(const Map& map_in);
    
    void reset();
    
    json get_path(const json input);
    
private:
    
    const double C_SPEED_LIMIT_MPH = 49.5;
    
    /** */
    void plan_behavior();
    
    /** */
    json generate_target_path();

    double getClosestElement(const std::vector<Traffic>& traffic) const;
    
    double getSpeedOfClosestElement(const std::vector<Traffic>& traffic, const double default_speed) const;
    
    // Reference to map
    const Map& map;
    
    // Internal storage of current vehicle state
    VehicleState state;
    
    // Internal storage of all sensed traffic elements
    std::vector<Traffic> sensed_traffic;
    
    double ref_vel;
    double ref_lane;
    
    double target_speed;
    unsigned target_lane;
    
    bool changing_lane;
    
};

#endif //_VEHICLE_H_

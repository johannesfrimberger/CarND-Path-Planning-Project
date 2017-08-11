#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "json.hpp"
#include <vector>

#include "Map.h"

// for convenience
using json = nlohmann::json;

/**  */
class Vehicle
{
public:
    
    Vehicle(const Map& map_in);
    
    json update(const json input);
    
private:
    
    const Map& map;
    
    int lane;
    double ref_vel;
    
};

#endif //_VEHICLE_H_

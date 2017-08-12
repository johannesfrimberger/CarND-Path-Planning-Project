#ifndef _TRAFFIC_H_
#define _TRAFFIC_H_

#include "json.hpp"

#include "Coordinates.h"

// for convenience
using json = nlohmann::json;

/**  */
class Traffic
{
public:
    
    Traffic(const json input);
    
private:
    
    const WorldCoordinates world;
    const WorldCoordinates speed;
    const FrenetCoordinates frenet;
    const unsigned id;
    
};

#endif //_TRAFFIC_H_

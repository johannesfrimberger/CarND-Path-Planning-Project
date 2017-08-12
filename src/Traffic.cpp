#include "Traffic.h"


Traffic::Traffic(const json input):
world(input[1], input[2]),
speed(input[3], input[4]),
frenet(input[5], input[6]),
id(input[0])
{
    
}

Traffic Traffic::simulate(const double dt) const
{
    // Copy current traffic object and update positions
    Traffic simulated_obj = *this;
    
    simulated_obj.world = simulated_obj.world + (speed * dt);
    simulated_obj.frenet = simulated_obj.frenet +
    FrenetCoordinates(simulated_obj.getSpeed() *  dt, 0);
    
    return simulated_obj;
}

void Traffic::addToRelevantList(std::vector<Traffic>& front, std::vector<Traffic>& rear, const VehicleState& ego) const
{
    const double distance = frenet.getDistance(ego.getFrenet());
    
    // In front
    if(distance > 0.0)
    {
        if(distance < 30.0)
        {
            front.push_back(*this);
        }
    }
    else
    {
        if(distance < -30.0)
        {
            rear.push_back(*this);
        }
    }
}

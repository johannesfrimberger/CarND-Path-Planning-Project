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

void Traffic::addToRelevantList(std::vector<RelativTraffic>& front, std::vector<RelativTraffic>& rear, const VehicleState& ego) const
{
    const RelativTraffic rel(*this, ego);

    // In front
    if(rel.isVehicleInFront())
    {
        if(rel.getDistance() < 200.0)
        {
            front.push_back(rel);
        }
    }
    else
    {
        if(rel.getDistance() < 50.0)
        {
            rear.push_back(rel);
        }
    }
}

RelativTraffic::RelativTraffic(const Traffic traffic, const VehicleState& ego):
Traffic(traffic)
{
    const double d = frenet.getDistance(ego.getFrenet());
    in_front = (d > 0.0);
    distance = fabs(d);
}

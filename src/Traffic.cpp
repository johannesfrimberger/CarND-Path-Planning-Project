#include "Traffic.h"


Traffic::Traffic(const json input):
world(input[1], input[2]),
speed(input[3], input[4]),
frenet(input[5], input[6]),
traffic_id(input[0])
{
}

Traffic Traffic::simulate(const double dt) const
{
    // Copy current traffic object and update positions
    Traffic simulated_obj = *this;
    
    // Simulate position at time dt with constant velocity
    simulated_obj.world = simulated_obj.world + (speed * dt);
    simulated_obj.frenet = simulated_obj.frenet +
    FrenetCoordinates(simulated_obj.getSpeed() *  dt, 0);
    
    return simulated_obj;
}

void Traffic::addToRelevantList(std::vector<RelativTraffic>& front, std::vector<RelativTraffic>& rear, const VehicleState& ego) const
{
    // Create RelativTraffic object
    const RelativTraffic rel(*this, ego);

    // Check distance and push it to front or rear list
    if(rel.getDistance() < 200.0)
    {
        if(rel.isVehicleInFront())
        {
            front.push_back(rel);
        }
        else
        {
            rear.push_back(rel);
        }
    }
}

RelativTraffic::RelativTraffic(const Traffic traffic, const VehicleState& ego):
Traffic(traffic)
{
    // Calulcate distance to referene object
    const double distance = frenet.getDistance(ego.getFrenet());
    
    // Vehicle is in front if distance is bigger than zero
    this->in_front = (distance > 0.0);
    
    // Store absolute distance
    this->distance = fabs(distance);
}

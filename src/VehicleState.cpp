#include "VehicleState.h"
#include "Tools.h"

using namespace std;

VehicleState::VehicleState():
world(0,0),
frenet(0,0)
{
}

void VehicleState::update(const json input)
{
    // Main car's localization Data
    world = WorldCoordinates(input[1]["x"], input[1]["y"]);
    frenet = FrenetCoordinates(input[1]["s"], input[1]["d"]);
    
    // Vehicle dynamics
    yaw = Tools::deg2rad(input[1]["yaw"]);
    speed = input[1]["speed"];
    
    // Previous path's end s and d values
    end_path_s = input[1]["end_path_s"];
    end_path_d = input[1]["end_path_d"];
    
    // Previous path data given to the Planner
    const auto previous_path_x = input[1]["previous_path_x"];
    const auto previous_path_y = input[1]["previous_path_y"];
    
    previous_path.clear();
    for(unsigned i = 0U; i < previous_path_x.size(); i++)
    {
        previous_path.push_back(WorldCoordinates(previous_path_x[i], previous_path_y[i]));
    }
}

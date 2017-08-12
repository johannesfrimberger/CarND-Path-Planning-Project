#include "Vehicle.h"
#include "Tools.h"

#include "spline.h"

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

Vehicle::Vehicle(const Map& map_in):
map(map_in)
{
    lane = 1;
    ref_vel = 0.0;
}

json Vehicle::generate_target_path(const double target_speed, const int target_lane)
{
    const unsigned prev_size = state.getNumberOfPreviousPathElements();

    vector<WorldCoordinates> pts;
    
    WorldCoordinates ref = state.getWorldCoordinats();
    double ref_yaw = state.getYaw();
    
    if(prev_size < 2)
    {
        WorldCoordinates prev_car = ref - WorldCoordinates(cos(ref_yaw), sin(ref_yaw));
        
        pts.push_back(prev_car);
        pts.push_back(ref);

    }
    else
    {
        ref = state.getPreviousPathElementReverse(0U);
        WorldCoordinates prev_car = state.getPreviousPathElementReverse(1U);

        ref_yaw = prev_car.getHeading(ref);
        
        pts.push_back(prev_car);
        pts.push_back(ref);
    }
    
    const double car_s = state.getFrenet().getS();
    
    pts.push_back(map.getXY(FrenetCoordinates(car_s+30, (2+(4*lane)))));
    pts.push_back(map.getXY(FrenetCoordinates(car_s+60, (2+(4*lane)))));
    pts.push_back(map.getXY(FrenetCoordinates(car_s+90, (2+(4*lane)))));
    
    for(int i = 0; i < pts.size();i++)
    {
        const WorldCoordinates shift = pts[i] - ref;
        const double shift_x = shift.getX();
        const double shift_y = shift.getY();
        
        pts[i] = WorldCoordinates((shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw)),
                                  (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw)));
    }
    
    vector<double> ptsx;
    vector<double> ptsy;
    Tools::splitWorldCoordinates(ptsx, ptsy, pts);
    tk::spline s;
    s.set_points(ptsx, ptsy);
    
    vector<WorldCoordinates> next_vals;
    
    for(auto el : state.getPreviousPath())
    {
        next_vals.push_back(el);
    }
    
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
    
    double x_add_on = 0.0;
    
    double max_acc = 0.224;
    if(false)
    {
        ref_vel -= max_acc;
    }
    else if(ref_vel < target_speed)
    {
        ref_vel += max_acc;
    }
    
    for(int i = 0; i < 50 - prev_size;i++)
    {
        const double N = (target_dist / (0.02*ref_vel/2.24));
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);
        
        x_add_on = x_point;
        
        const double x_ref = x_point;
        const double y_ref = y_point;
        
        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
        next_vals.push_back(ref + WorldCoordinates(x_point, y_point));
    }
    
    // Split up path in x,y vector for json output
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    Tools::splitWorldCoordinates(next_x_vals, next_y_vals, next_vals);
    
    json msgJson;
    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;
    
    return msgJson;
}

json Vehicle::get_path(const json input)
{
    // Update current vehicle status
    state.update(input);
    
    // Update sensor fusion data (a list of all other cars on the same side of the road)
    auto sensor_fusion = input[1]["sensor_fusion"];
    sensed_traffic.clear();
    for(unsigned i = 0U; i < sensor_fusion.size(); i++)
    {
        sensed_traffic.push_back(Traffic(sensor_fusion[i]));
    }
    
    // 

    // Generate target path and return it in json format
    return generate_target_path(49.5, 1);
}

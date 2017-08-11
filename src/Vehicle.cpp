#include "Vehicle.h"
#include "Tools.h"

#include "spline.h"

using namespace std;

Vehicle::Vehicle(const Map& map_in):
map(map_in)
{
    lane = 1;
    ref_vel = 0.0;
}

json Vehicle::update(const json input)
{
    
    
    // Main car's localization Data
    double car_x = input[1]["x"];
    double car_y = input[1]["y"];
    double car_s = input[1]["s"];
    double car_d = input[1]["d"];
    double car_yaw = input[1]["yaw"];
    double car_speed = input[1]["speed"];
    
    // Previous path data given to the Planner
    auto previous_path_x = input[1]["previous_path_x"];
    auto previous_path_y = input[1]["previous_path_y"];
    // Previous path's end s and d values
    double end_path_s = input[1]["end_path_s"];
    double end_path_d = input[1]["end_path_d"];
    
    int prev_size = previous_path_x.size();
    
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = input[1]["sensor_fusion"];
    
    vector<double> ptsx;
    vector<double> ptsy;
    
    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = Tools::deg2rad(car_yaw);
    
    if(prev_size < 2)
    {
        double prev_car_x = ref_x - cos(ref_yaw);
        double prev_car_y = ref_y - sin(ref_yaw);
        
        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);
        
        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    }
    else
    {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        
        double prev_car_x = previous_path_x[prev_size - 2];
        double prev_car_y = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
        
        ptsx.push_back(prev_car_x);
        ptsx.push_back(ref_x);
        
        ptsy.push_back(prev_car_y);
        ptsy.push_back(ref_y);
    }
    
    WorldCoordinates next_wp0 = map.getXY(FrenetCoordinates(car_s+30, (2+(4*lane))));
    WorldCoordinates next_wp1 = map.getXY(FrenetCoordinates(car_s+60, (2+(4*lane))));
    WorldCoordinates next_wp2 = map.getXY(FrenetCoordinates(car_s+90, (2+(4*lane))));
    
    ptsx.push_back(next_wp0.getX());
    ptsx.push_back(next_wp1.getX());
    ptsx.push_back(next_wp2.getX());
    
    ptsy.push_back(next_wp0.getX());
    ptsy.push_back(next_wp1.getX());
    ptsy.push_back(next_wp2.getX());
    
    for(int i = 0; i < ptsx.size();i++)
    {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;
        
        ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
    }
    
    tk::spline s;
    
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    
    s.set_points(ptsx, ptsy);
    
    for(auto el : previous_path_x)
    {
        next_x_vals.push_back(el);
    }
    
    for(auto el : previous_path_y)
    {
        next_y_vals.push_back(el);
    }
    
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
    
    double x_add_on = 0.0;
    
    double target_speed = 49.5;
    
    double max_acc = 0.224;
    if(false)
    {
        ref_vel -= max_acc;
    }
    else if(ref_vel < target_speed)
    {
        ref_vel += max_acc;
    }
    
    for(int i = 0; i < 50 - previous_path_x.size();i++)
    {
        double N = (target_dist / (0.02*ref_vel/2.24));
        double x_point = x_add_on + target_x/N;
        double y_point = s(x_point);
        
        x_add_on = x_point;
        
        double x_ref = x_point;
        double y_ref = y_point;
        
        x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
        
        x_point += ref_x;
        y_point += ref_y;
        
        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    
    json msgJson;

    msgJson["next_x"] = next_x_vals;
    msgJson["next_y"] = next_y_vals;
    
    return msgJson;
}

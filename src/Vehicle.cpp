#include "Vehicle.h"
#include "Tools.h"

#include "spline.h"

using namespace std;

Vehicle::Vehicle(const Map& map_in):
map(map_in)
{
    reset();
}

void Vehicle::reset()
{
    std::cout << "Reset internal states" << std::endl;
    ref_vel = 0.0;
    target_lane = 1;
    ref_lane = 1;
}

double Vehicle::getSpeedOfClosestElement(const std::vector<Traffic>& traffic, const double default_speed) const
{
    double speed = -1;
    
    if(traffic.size() > 0U)
    {
        double min_distance = 1000;
        
        for(auto el : traffic)
        {
            const double distance = el.getFrenet().getDistance(state.getFrenet());
            if(distance < min_distance)
            {
                min_distance = distance;
                speed = el.getSpeed();
            }
        }
    }
    
    if(speed > 0)
    {
        return speed;
    }
    else
    {
        return default_speed;
    }
}

json Vehicle::generate_target_path()
{
    const unsigned prev_size = state.getNumberOfPreviousPathElements();

    // Define reference position and reference yaw rate
    WorldCoordinates ref = state.getWorldCoordinats();
    double ref_yaw = state.getYaw();
    
    //
    vector<WorldCoordinates> pts;
    
    // Check if previous path is available
    if(prev_size < 2)
    {
        // Estimate previous position of the car
        WorldCoordinates prev_car = ref - WorldCoordinates(cos(ref_yaw), sin(ref_yaw));
        
        pts.push_back(prev_car);
        pts.push_back(ref);
    }
    else
    {
        // Define reference position as last element of the previous path
        ref = state.getPreviousPathElementReverse(0U);
        
        // Estimate reference yaw/heading with last two points of previous path
        const WorldCoordinates prev_car = state.getPreviousPathElementReverse(1U);
        ref_yaw = prev_car.getHeading(ref);
        
        pts.push_back(prev_car);
        pts.push_back(ref);
    }
    
    // Short access to current s frenet coordinate
    const double car_s = state.getFrenet().getS();
    
    // Adjust current velocity
    double max_steering = 0.05;
    
    std::cout << "Ref " << ref_lane << " target " << target_lane << std::endl;
    
    if(ref_lane > target_lane)
    {
        ref_lane -= max_steering;
    }
    else if(ref_lane < target_lane)
    {
        ref_lane += max_steering;
    }
    
    // Saturate 
    if(ref_lane < 0)
    {
        ref_lane = 0;
    }
    else if(ref_lane > 2)
    {
        ref_lane = 2;
    }
    
    // Add points along the map for interpolation
    pts.push_back(map.getXY(FrenetCoordinates(car_s+30, (2+(4*ref_lane)))));
    pts.push_back(map.getXY(FrenetCoordinates(car_s+60, (2+(4*ref_lane)))));
    pts.push_back(map.getXY(FrenetCoordinates(car_s+90, (2+(4*ref_lane)))));
    
    // Convert all points to vehicle CoSy
    for(int i = 0; i < pts.size();i++)
    {
        const WorldCoordinates shift = pts[i] - ref;
        const double shift_x = shift.getX();
        const double shift_y = shift.getY();
        
        pts[i] = WorldCoordinates((shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw)),
                                  (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw)));
    }
    
    // Create spline over next waypoints in vehicle CoSy
    vector<double> ptsx;
    vector<double> ptsy;
    Tools::splitWorldCoordinates(ptsx, ptsy, pts);
    tk::spline s;
    s.set_points(ptsx, ptsy);
    
    // Generate vector of next waypoints and init with previous waypoints
    vector<WorldCoordinates> next_vals;
    for(auto el : state.getPreviousPath())
    {
        next_vals.push_back(el);
    }
    
    // Estimate target
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
    
    double x_add_on = 0.0;
    
    // Adjust current velocity
    double max_acc = 0.224;
    if(ref_vel > target_speed)
    {
        ref_vel -= max_acc;
    }
    else if(ref_vel < target_speed)
    {
        ref_vel += max_acc;
    }
    
    const unsigned target_n_waypoints = 50;
    
    for(int i = 0; i < target_n_waypoints - prev_size;i++)
    {
        const double N = (target_dist / (0.02 * ref_vel));
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

void Vehicle::plan_behavior()
{
    // Find closest traffic in each lane
    std::vector<Traffic> middle_lane_in_front;
    std::vector<Traffic> left_lane_in_front;
    std::vector<Traffic> right_lane_in_front;
    std::vector<Traffic> middle_lane_in_back;
    std::vector<Traffic> left_lane_in_back;
    std::vector<Traffic> right_lane_in_back;

    for(auto traffic : sensed_traffic)
    {
        Traffic simulated_traffic = traffic.simulate(state.getNumberOfPreviousPathElements() * 0.02);
        switch(traffic.getFrenet().getLane())
        {
            case 0U:
                traffic.addToRelevantList(left_lane_in_front, left_lane_in_back, state);
                break;
            case 1U:
                traffic.addToRelevantList(middle_lane_in_front, middle_lane_in_back, state);
                break;
            case 2U:
                traffic.addToRelevantList(right_lane_in_front, right_lane_in_back, state);
                break;
            default:
                std::cout << "invalid lane id received" << std::endl;
        }
    }
    
    std::cout << "Left lane has " << left_lane_in_front.size() << " vehicles in front" << std::endl;
    std::cout << "Middle lane has " << middle_lane_in_front.size() << " vehicles in front" << std::endl;
    std::cout << "Right lane has " << right_lane_in_front.size() << " vehicles in front" << std::endl;
    
    const unsigned current_lane = state.getFrenet().getLane();
    
    if(changing_lane)
    {
        // If we are currently changing lane check if new lane has already been reached
        if(fabs(state.getFrenet().getD() - target_lane) < 0.1)
        {
            changing_lane = false;
        }
    }
    // If we are not changing lane look if there is a better lane
    else
    {
        
    }
    
    // Adjust speed to speed in current lane
    switch(current_lane)
    {
        case 0U:
            target_speed = getSpeedOfClosestElement(left_lane_in_front, target_speed);
            break;
        case 1U:
            target_speed = getSpeedOfClosestElement(middle_lane_in_front, target_speed);
            break;
        case 2U:
            target_speed = getSpeedOfClosestElement(right_lane_in_front, target_speed);
            break;
        default:
            break;
    }
}

json Vehicle::get_path(const json input)
{
    //
    target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
    
    
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
    plan_behavior();

    // Generate target path and return it in json format
    return generate_target_path();
}

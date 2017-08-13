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
    pathPlannerState = KEEP_LANE;
}

std::string Vehicle::ppSToString(const Vehicle::PathPlannerStateType state) const
{
    switch(state)
    {
        case KEEP_LANE:
            return "KEEP_LANE";
        case PREPARE_CHANGE_LEFT:
            return "PREPARE_CHANGE_LEFT";
        case PREPARE_CHANGE_RIGHT:
            return "PREPARE_CHANGE_RIGHT";
        case CHANGE_LEFT:
            return "CHANGE_LEFT";
        case CHANGE_RIGHT:
            return "CHANGE_RIGHT";
        case PREPARE_CHANGE_2ndLEFT:
            return "PREPARE_CHANGE_2ndLEFT";
        case PREPARE_CHANGE_2ndRIGHT:
            return "PREPARE_CHANGE_2ndRIGHT";
        case CHANGE_2ndLEFT:
            return "CHANGE_2ndLEFT";
        case CHANGE_2ndRIGHT:
            return "CHANGE_2ndRIGHT";
        case EXIT_STRATEGY:
            return "EXIT_STRATEGY";
        default:
            return "Invalid";
    }
}

bool Vehicle::isChangeToLeftLaneBenefitial()
{
    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    const double closestElementLeft = getClosestElement(left_lane_in_front);
    const double closestElementInBack = getClosestElement(left_lane_in_back);
    const int current_lane = state.getFrenet().getLane();

    const double diff = closestElementLeft - closestElementThisLane;
    std::cout << "Diff " << diff << std::endl;
    return (diff > C_DIST_HYST_LANE_CHANGE) && newLaneIsValid(current_lane - 1) && (closestElementInBack > C_DIST_TRAFFIC_BACK);
}

bool Vehicle::isChangeToRightLaneBenefitial()
{
    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    const double closestElementRight = getClosestElement(right_lane_in_front);
    const double closestElementInBack = getClosestElement(right_lane_in_back);
    const int current_lane = state.getFrenet().getLane();
    
    const double diff = closestElementRight - closestElementThisLane;
    std::cout << "Diff " << diff << std::endl;
    return (diff > C_DIST_HYST_LANE_CHANGE) && newLaneIsValid(current_lane + 1) && (closestElementInBack > C_DIST_TRAFFIC_BACK);
}

double Vehicle::getClosestElement(const std::vector<RelativTraffic>& traffic) const
{
    double min_distance = 1000;
    
    for(auto el : traffic)
    {
        const double distance = el.getDistance();
        if(distance < min_distance)
        {
            min_distance = distance;
        }
    }
    
    return min_distance;
}

double Vehicle::getSpeedOfClosestElement(const std::vector<RelativTraffic>& traffic, const double default_speed) const
{
    double speed = -1;
    
    if(traffic.size() > 0U)
    {
        double min_distance = 1000;
        
        for(auto el : traffic)
        {
            const double distance = el.getDistance();
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
    
    // Adjust ref_lane slowely to target_lane to avoid too much acceleration
    double max_steering = 0.02;
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

Vehicle::PathPlannerStateType Vehicle::updateKeepLane()
{
    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    
    // Do nothing and stay in lane if all vehicles are more than 50m in front
    if(closestElementThisLane > C_DIST_STAY_IN_LANE)
    {
        target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
        //std::cout << "Stay in lane with "<< target_speed << " m/s, closest element is " << closestElementThisLane << " away" << std::endl;
        
        return KEEP_LANE;
    }
    
    // Check which is the best lane to change to
    bool leftBenefetial = isChangeToLeftLaneBenefitial();
    bool rightBenefetial = isChangeToRightLaneBenefitial();
    
    if(leftBenefetial && rightBenefetial)
    {
        const double closestElementLeft = getClosestElement(left_lane_in_front);
        const double closestElementRight = getClosestElement(right_lane_in_front);
        
        if(closestElementLeft < closestElementRight)
        {
            leftBenefetial = false;
        }
    }
    
    if(leftBenefetial)
    {
        return PREPARE_CHANGE_LEFT;
    }
    else if(rightBenefetial)
    {
        return PREPARE_CHANGE_RIGHT;
    }
    
    if(closestElementThisLane < C_DIST_REDUCE_SPEED)
    {
        // If we have to stay in lane adapt speed
        target_speed = getSpeedOfClosestElement(this_lane_in_front, target_speed);
        std::cout << "Adapt speed to target " << target_speed << std::endl;
    }
    else
    {
        target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
    }
    
    return KEEP_LANE;
}

Vehicle::PathPlannerStateType Vehicle::updatePrepareChangeLeft()
{
    // Check if lane change is safe
    const double closestElementInBack = getClosestElement(left_lane_in_back);
    
    if(!isChangeToLeftLaneBenefitial())
    {
        std::cout << "Change to left lane is no longer benefitial" << std::endl;
        return KEEP_LANE;
    }
    
    if((closestElementInBack > C_DIST_TRAFFIC_BACK))
    {
        target_lane -= 1U;
        return CHANGE_LEFT;
    }
    
    std::cout << "Waiting to change to left lane due to element in back at " << closestElementInBack << std::endl;
    
    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    if(closestElementThisLane < C_DIST_REDUCE_SPEED)
    {
        // If we have to stay in lane adapt speed
        target_speed = getSpeedOfClosestElement(this_lane_in_front, target_speed);
        std::cout << "Adapt speed to target " << target_speed << std::endl;
    }
    else
    {
        target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
    }
    
    // State is eithe
    return pathPlannerState;
}

Vehicle::PathPlannerStateType Vehicle::updatePrepareChangeRight()
{
    // Check if lane change is safe
    const double closestElementInBack = getClosestElement(right_lane_in_back);
    
    if(!isChangeToRightLaneBenefitial())
    {
        std::cout << "Change to right lane is no longer benefitial" << std::endl;
        return KEEP_LANE;
    }
    
    if((closestElementInBack > C_DIST_TRAFFIC_BACK))
    {
        target_lane += 1U;
        return CHANGE_RIGHT;
    }
    
    std::cout << "Waiting to change to right lane due to element in back at " << closestElementInBack << std::endl;
    
    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    if(closestElementThisLane < C_DIST_REDUCE_SPEED)
    {
        // If we have to stay in lane adapt speed
        target_speed = getSpeedOfClosestElement(this_lane_in_front, target_speed);
        std::cout << "Adapt speed to target " << target_speed << std::endl;
    }
    else
    {
        target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
    }
    
    // State is eithe
    return pathPlannerState;
}

Vehicle::PathPlannerStateType Vehicle::updateChangeLeft()
{
    // If we are currently changing lane check if new lane has already been reached
    if(fabs(state.getFrenet().getLane() - target_lane) < 0.1)
    {
        return KEEP_LANE;
    }
    else
    {
        std::cout << "Changing Lane " << fabs(ref_lane - target_lane) << " " << target_lane << std::endl;
        return CHANGE_LEFT;
    }
}

Vehicle::PathPlannerStateType Vehicle::updateChangeRight()
{
    // If we are currently changing lane check if new lane has already been reached
    if(fabs(state.getFrenet().getLane() - target_lane) < 0.1)
    {
        return KEEP_LANE;
    }
    else
    {
        std::cout << "Changing Lane " << fabs(ref_lane - target_lane) << " " << target_lane << std::endl;
        return CHANGE_RIGHT;
    }
}

void Vehicle::plan_behavior()
{
    // Find relevant traffic in each lane
    left_lane_in_front.clear();
    left_lane_in_back.clear();
    left_2nd_lane_in_front.clear();
    left_2nd_lane_in_back.clear();
    this_lane_in_front.clear();
    this_lane_in_back.clear();
    right_lane_in_front.clear();
    right_lane_in_back.clear();
    right_2nd_lane_in_front.clear();
    right_2nd_lane_in_back.clear();
    
    
    const int current_lane = state.getFrenet().getLane();

    for(auto traffic : sensed_traffic)
    {
        Traffic simulated_traffic = traffic.simulate(state.getNumberOfPreviousPathElements() * 0.02);
        const int lane_diff = current_lane - static_cast<int>(traffic.getFrenet().getLane());
        switch(lane_diff)
        {
            case -2:
                traffic.addToRelevantList(right_2nd_lane_in_front, right_2nd_lane_in_back, state);
                break;
            case -1:
                traffic.addToRelevantList(right_lane_in_front, right_lane_in_back, state);
                break;
            case 0:
                traffic.addToRelevantList(this_lane_in_front, this_lane_in_back, state);
                break;
            case 1:
                traffic.addToRelevantList(left_lane_in_front, left_lane_in_back, state);
                break;
            case 2:
                traffic.addToRelevantList(left_2nd_lane_in_front, left_2nd_lane_in_back, state);
                break;
            default:
                std::cout << "Invalid lane id of a traffic element received" << std::endl;
        }
    }

    /*
    std::cout << "Left lane has " << left_lane_in_back.size() << " vehicles in back" << std::endl;
    std::cout << "This lane has " << this_lane_in_back.size() << " vehicles in back" << std::endl;
    std::cout << "Right lane has " << right_lane_in_back.size() << " vehicles in back" << std::endl;
    */
    
    PathPlannerStateType new_state = KEEP_LANE;
    
    switch(pathPlannerState)
    {
        case KEEP_LANE:
            new_state = updateKeepLane();
            break;
        case PREPARE_CHANGE_LEFT:
        case PREPARE_CHANGE_2ndLEFT:
            new_state = updatePrepareChangeLeft();
            break;
        case PREPARE_CHANGE_RIGHT:
        case PREPARE_CHANGE_2ndRIGHT:
            new_state = updatePrepareChangeRight();
            break;
        case CHANGE_LEFT:
            new_state = updateChangeLeft();
            break;
        case CHANGE_RIGHT:
            new_state = updateChangeRight();
            break;
        default:
            std::cout << "Invalid pathPlannerState, set to KEEP_LANE" << std::endl;
            break;
    }
    
    if(pathPlannerState != new_state)
    {
        std::cout << "Update state from " << ppSToString(pathPlannerState) << " to " << ppSToString(new_state) << std::endl;
        pathPlannerState = new_state;
    }
    else
    {
        std::cout << "Stay in state: " << ppSToString(pathPlannerState) << std::endl;
    }
    
    /*
     
     const unsigned current_lane = state.getFrenet().getLane();
    if(changing_lane)
    {
        // If we are currently changing lane check if new lane has already been reached
        if(fabs(ref_lane- target_lane) < 0.1)
        {
            changing_lane = false;
        }
        else
        {
            std::cout << "Changing Lane " << fabs(ref_lane - target_lane) << std::endl;
        }
    }
    // If we are not already changing lane look if there is a better lane
    else
    {
        unsigned updated_lane = target_lane;
        switch(current_lane)
        {
            case 0U:
                // Consider middle and right lane for change
                if(left_lane_in_front.size() > 0)
                {
                    const double distance = getClosestElement(left_lane_in_front);
                    const double distance_middle = getClosestElement(middle_lane_in_front);
                    const double distance_right = getClosestElement(right_lane_in_front);
                    
                    if(distance_middle > distance)
                    {
                        std::cout << "Swicht to middle lane" << std::endl;
                        updated_lane = 1U;
                    }
                }
                break;
            case 1U:
                // Consider left and right lane for change
                if(middle_lane_in_front.size() > 0)
                {
                    const double distance = getClosestElement(middle_lane_in_front);
                    const double distance_left = getClosestElement(left_lane_in_front);
                    const double distance_right = getClosestElement(right_lane_in_front);
                    
                    if(distance_left > distance_right)
                    {
                        if(distance_left > distance)
                        {
                            std::cout << "Swicht to left lane" << std::endl;
                            updated_lane = 0U;
                        }
                        else if(distance_right > distance)
                        {
                            std::cout << "Swicht to right lane" << std::endl;
                            updated_lane = 2U;
                        }
                    }
                    else
                    {
                        if(distance_right > distance)
                        {
                            std::cout << "Swicht to right lane" << std::endl;
                            updated_lane = 2U;
                        }
                        else if(distance_left > distance)
                        {
                            std::cout << "Swicht to left lane" << std::endl;
                            updated_lane = 0U;
                        }
                    }
                    
                    
                }
                break;
            case 2U:
                // Consider left and middle lane for change
                if(right_lane_in_front.size() > 0)
                {
                    const double distance = getClosestElement(right_lane_in_front);
                    const double distance_middle = getClosestElement(middle_lane_in_front);
                    const double distance_left = getClosestElement(left_lane_in_front);
                    
                    if(distance_middle > distance)
                    {
                        std::cout << "Swicht to middle lane" << std::endl;
                        updated_lane = 1U;
                    }
                }
                break;
            default:
                std::cout << "Ego Vehicle in invalid lane" << std::endl;
                break;
        }
        
        if(target_lane != updated_lane)
        {
            
            target_lane = updated_lane;
            changing_lane = true;
        }
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
    
    */
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
    plan_behavior();

    // Generate target path and return it in json format
    return generate_target_path();
}

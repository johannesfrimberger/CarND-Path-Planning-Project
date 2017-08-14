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

void Vehicle::setMaxSpeed()
{
    target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
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

bool Vehicle::isLaneChangeBenefitial(const std::vector<RelativTraffic>& traffic_new_lane_front, const std::vector<RelativTraffic>& traffic_new_lane_back, const int lane_change) const
{
    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    const double closestElementLeft = getClosestElement(traffic_new_lane_front);
    const double closestElementInBack = getClosestElement(traffic_new_lane_back);
    const double speedOfClosestElementInBack = getSpeedOfClosestElement(traffic_new_lane_back, state.getSpeed());
    const int current_lane = state.getFrenet().getLane();
    
    const double diff = closestElementLeft - closestElementThisLane;
    
    const double diff_speed = (speedOfClosestElementInBack - state.getSpeed());
    
    return (diff > C_DIST_HYST_LANE_CHANGE) && newLaneIsValid(current_lane + lane_change) && ((closestElementInBack > C_DIST_TRAFFIC_BACK_SAFE) || ((closestElementInBack > C_DIST_TRAFFIC_BACK) && (diff_speed < C_DIFF_SPEED_APPROACHING_BACK)));
}

bool Vehicle::isChangeToLeftLaneBenefitial() const
{
    return isLaneChangeBenefitial(left_lane_in_front, left_lane_in_back, -1);
}

bool Vehicle::isChangeToRightLaneBenefitial() const
{
    return isLaneChangeBenefitial(right_lane_in_front, right_lane_in_back, 1);
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
    // Initialize speed with negative number
    double speed = -1;
    
    // Find speed of closest element in list
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
    
    // If speed was changed from init value return this
    if(speed > 0)
    {
        return speed;
    }
    // Otherwise return default_speed
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
    
    // Vectore to store upcoming pts
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
    // Adjust current velocity
    if(fabs(ref_vel - target_speed) > C_MAX_ACC - 0.001)
    {
        if(ref_vel > target_speed)
        {
            ref_vel -= C_MAX_ACC;
        }
        else if(ref_vel < target_speed)
        {
            ref_vel += C_MAX_ACC;
        }
    }
    else if(fabs(ref_lane - target_lane) > C_MAX_STEERING - 0.001)
    {
        if(ref_lane > target_lane)
        {
            ref_lane -= C_MAX_STEERING;
        }
        else if(ref_lane < target_lane)
        {
            ref_lane += C_MAX_STEERING;
        }
    }
    
    // Saturate ref_lane
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
        setMaxSpeed();
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
        target_speed = getSpeedOfClosestElement(this_lane_in_front, target_speed) - 1.0;
    }
    else
    {
        setMaxSpeed();
    }
    
    return KEEP_LANE;
}

Vehicle::PathPlannerStateType Vehicle::updatePrepareChangeLeft()
{
    // Check if lane change is safe
    const double closestElementInBack = getClosestElement(left_lane_in_back);
    
    if(!isChangeToLeftLaneBenefitial())
    {
        return KEEP_LANE;
    }
    
    if((closestElementInBack > C_DIST_TRAFFIC_BACK))
    {
        target_lane -= 1U;
        target_speed -= .5;
        return CHANGE_LEFT;
    }
    
    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    if(closestElementThisLane < C_DIST_REDUCE_SPEED)
    {
        // If we have to stay in lane adapt speed
        target_speed = getSpeedOfClosestElement(this_lane_in_front, target_speed);
    }
    else
    {
        target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
    }
    
    return pathPlannerState;
}

Vehicle::PathPlannerStateType Vehicle::updatePrepareChangeRight()
{
    // Check if lane change is safe
    const double closestElementInBack = getClosestElement(right_lane_in_back);
    
    if(!isChangeToRightLaneBenefitial())
    {
        return KEEP_LANE;
    }
    
    if((closestElementInBack > C_DIST_TRAFFIC_BACK))
    {
        target_lane += 1U;
        target_speed -= .5f;
        return CHANGE_RIGHT;
    }

    const double closestElementThisLane = getClosestElement(this_lane_in_front);
    if(closestElementThisLane < C_DIST_REDUCE_SPEED)
    {
        // If we have to stay in lane adapt speed
        target_speed = getSpeedOfClosestElement(this_lane_in_front, target_speed);
    }
    else
    {
        target_speed = Tools::mph2mps(C_SPEED_LIMIT_MPH);
    }
    
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
        return CHANGE_RIGHT;
    }
}

void Vehicle::update_plath_planner()
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
        //Traffic simulated_traffic = traffic.simulate(state.getNumberOfPreviousPathElements() * 0.02);
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
                //std::cout << "Invalid lane of a traffic element " << traffic.getId() << std::endl;
                break;
        }
    }

    // Update PathPlanner state
    PathPlannerStateType new_state = KEEP_LANE;
    
    switch(pathPlannerState)
    {
        case KEEP_LANE:
            new_state = updateKeepLane();
            break;
        case PREPARE_CHANGE_LEFT:
            new_state = updatePrepareChangeLeft();
            break;
        case PREPARE_CHANGE_RIGHT:
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
    
    // Update internal state
    if(pathPlannerState != new_state)
    {
        std::cout << "Update state from " << ppSToString(pathPlannerState) << " to " << ppSToString(new_state) << std::endl;
        pathPlannerState = new_state;
    }
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
    
    // Update target_speed and target_lane for current situation
    update_plath_planner();

    // Generate target path and return it in json format
    return generate_target_path();
}

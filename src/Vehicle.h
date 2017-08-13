#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "json.hpp"
#include <vector>

#include "Map.h"
#include "Traffic.h"
#include "VehicleState.h"

using json = nlohmann::json;

/**  */
class Vehicle
{
public:
    
    Vehicle(const Map& map_in);
    
    void reset();
    
    json get_path(const json input);
    
private:
    
    enum PathPlannerStateType
    {
        KEEP_LANE,
        PREPARE_CHANGE_LEFT,
        PREPARE_CHANGE_RIGHT,
        CHANGE_LEFT,
        CHANGE_RIGHT,
        PREPARE_CHANGE_2ndLEFT,
        PREPARE_CHANGE_2ndRIGHT,
        CHANGE_2ndLEFT,
        CHANGE_2ndRIGHT,
        EXIT_STRATEGY
    };
    
    const double C_SPEED_LIMIT_MPH = 49.5;
    const double C_DIST_STAY_IN_LANE = 50.0;
    const double C_DIST_REDUCE_SPEED = 25.0;
    const double C_DIST_TRAFFIC_BACK = 8.0;
    const double C_DIST_HYST_LANE_CHANGE = 8.0;
    
    /** */
    void plan_behavior();
    
    /** */
    json generate_target_path();
    
    PathPlannerStateType updateKeepLane();
    PathPlannerStateType updatePrepareChangeLeft();
    PathPlannerStateType updateChangeLeft();
    PathPlannerStateType updatePrepareChangeRight();
    PathPlannerStateType updateChangeRight();
    
    bool isChangeToLeftLaneBenefitial();
    bool isChangeToRightLaneBenefitial();
    

    double getClosestElement(const std::vector<RelativTraffic>& traffic) const;
    
    double getSpeedOfClosestElement(const std::vector<RelativTraffic>& traffic, const double default_speed) const;
    
    bool newLaneIsValid(const int new_lane) const
    {
        
        return (new_lane >= 0) && (new_lane <= 2);
    }
    
    void setMaxSpeed();
    
    // Convert PathPlannerStateType to string
    std::string ppSToString(const PathPlannerStateType state) const;
    
    // Reference to map
    const Map& map;
    
    // Internal storage of current vehicle state
    VehicleState state;
    
    // Internal storage of all sensed traffic elements
    std::vector<Traffic> sensed_traffic;
    
    // Internal storage of processed traffic
    std::vector<RelativTraffic> left_lane_in_front;
    std::vector<RelativTraffic> left_lane_in_back;
    std::vector<RelativTraffic> left_2nd_lane_in_front;
    std::vector<RelativTraffic> left_2nd_lane_in_back;
    std::vector<RelativTraffic> this_lane_in_front;
    std::vector<RelativTraffic> this_lane_in_back;
    std::vector<RelativTraffic> right_lane_in_front;
    std::vector<RelativTraffic> right_lane_in_back;
    std::vector<RelativTraffic> right_2nd_lane_in_front;
    std::vector<RelativTraffic> right_2nd_lane_in_back;
    
    double ref_vel;
    double ref_lane;
    
    double target_speed;
    unsigned target_lane;
    
    PathPlannerStateType pathPlannerState;
};

#endif //_VEHICLE_H_

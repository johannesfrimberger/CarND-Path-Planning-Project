#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include "json.hpp"
#include <vector>

#include "Map.h"
#include "Traffic.h"
#include "VehicleState.h"

using json = nlohmann::json;

/** Vehicle class implements all sensing and path planning */
class Vehicle
{
public:
    
    /** Explicit Constructor */
    explicit Vehicle(const Map& map_in);
    
    /** Reset all internal states */
    void reset();
    
    /** Return smooth path for current driving situation */
    json get_path(const json input);
    
private:
    
    // Speed limit that should not be exceeded
    const double C_SPEED_LIMIT_MPH = 49.5;
    
    // Consider lane change only if the distance to next vehicle is smaller than this
    const double C_DIST_STAY_IN_LANE = 50.0;
    
    // When distance is smaller than this reduce speed
    const double C_DIST_REDUCE_SPEED = 25.0;
    
    // Save distance to vehicle in the back for lane change
    const double C_DIST_TRAFFIC_BACK = 8.0;
    
    // Distance the new lane is at least better
    const double C_DIST_HYST_LANE_CHANGE = 8.0;
    
    /** State of path planning algorithm */
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
    
    /** Update path planner */
    void update_plath_planner();
    
    /** Generate target path from current requests*/
    json generate_target_path();
    
    /** Update state and velocity + lane in state KEEP_LANE */
    PathPlannerStateType updateKeepLane();
    
    /** Update state and velocity + lane in state PREPARE_CHANGE_LEFT */
    PathPlannerStateType updatePrepareChangeLeft();
    
    /** Update state and velocity + lane in state CHANGE_LEFT */
    PathPlannerStateType updateChangeLeft();
    
    /** Update state and velocity + lane in state PREPARE_CHANGE_RIGHT */
    PathPlannerStateType updatePrepareChangeRight();
    
    /** Update state and velocity + lane in state CHANGE_RIGHT */
    PathPlannerStateType updateChangeRight();
    
    /** Check if change to new lane is possible and benefitial */
    bool isLaneChangeBenefitial(const std::vector<RelativTraffic>& traffic_new_lane, const std::vector<RelativTraffic>& traffic_new_lane_back, const int lane_change) const;
    
    /** Check if change to left lane is possible and benefitial */
    bool isChangeToLeftLaneBenefitial() const;
    
    /** Check if change to right lane is possible and benefitial */
    bool isChangeToRightLaneBenefitial() const;

    /** Return distance to closest element in vector */
    double getClosestElement(const std::vector<RelativTraffic>& traffic) const;
    
    /** Return speed of closest element in vector */
    double getSpeedOfClosestElement(const std::vector<RelativTraffic>& traffic, const double default_speed) const;
    
    /** Check if new lane is valid (0-2) */
    bool newLaneIsValid(const int new_lane) const
    {
        
        return (new_lane >= 0) && (new_lane <= 2);
    }
    
    /** Set target_speed to speed limit (maximum speed) */
    void setMaxSpeed();
    
    /** Convert PathPlannerStateType to string */
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
    
    // Current velocity and lane
    double ref_vel;
    double ref_lane;
    
    // Target velocity and lane
    double target_speed;
    unsigned target_lane;
    
    // Internal state of path planner
    PathPlannerStateType pathPlannerState;
};

#endif //_VEHICLE_H_

#include <fstream>
#include <sstream>
#include <math.h>

#include "Map.h"

using namespace std;

Map::Map(std::string filename)
{
    // Clear waypoints
    waypoints.clear();
    
    // Read map from text file
    ifstream in_map_(filename, ifstream::in);
    
    string line;
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints.push_back(Waypoint(x,y,s,d_x, d_y));
    }
}

unsigned Map::getClosestWaypoint(const WorldCoordinates& pos) const
{
    double closestLen = 100000; //large number
    unsigned closestWaypoint = 0;
    
    for(unsigned i = 0; i < waypoints.size(); i++)
    {
        double dist = pos.getDistance(waypoints[i].getWorldCoordinats());
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
        
    }
    
    return closestWaypoint;
}

unsigned Map::getNextWaypoint(const WorldCoordinates& pos, const double theta) const
{
    unsigned closest_waypoint = getClosestWaypoint(pos);
    
    const WorldCoordinates map_position = waypoints[closest_waypoint].getWorldCoordinats();
    const double heading = pos.getHeading(map_position);
    
    double angle = fabs(theta-heading);
    
    if(angle > M_PI/4)
    {
        closest_waypoint++;
    }
    
    return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
FrenetCoordinates Map::getFrenet(const WorldCoordinates& pos, const double theta) const
{
    const unsigned id_next_wp = getNextWaypoint(pos, theta);
    
    int id_prev_wp;
    id_prev_wp = id_next_wp - 1;
    if(id_next_wp == 0)
    {
        id_prev_wp  = waypoints.size()-1;
    }
    
    const WorldCoordinates next_wp = waypoints[id_next_wp].getWorldCoordinats();
    const WorldCoordinates prev_wp = waypoints[id_prev_wp].getWorldCoordinats();
    
    const WorldCoordinates n_x = next_wp - prev_wp;
    const WorldCoordinates x_x = pos - prev_wp;
    
    // find the projection of x onto n
    const WorldCoordinates proj_norm = n_x.getProjection(x_x);
    double frenet_d = proj_norm.getDistance(x_x);
    
    //see if d value is positive or negative by comparing it to a center point
    const WorldCoordinates center = WorldCoordinates(1000, 2000) - prev_wp;
    const double centerToPos = center.getDistance(x_x);
    const double centerToRef = center.getDistance(proj_norm);
    
    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    
    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < id_prev_wp; i++)
    {
        frenet_s += waypoints[i].getWorldCoordinats().getDistance(waypoints[i+1].getWorldCoordinats());
    }
    
    frenet_s += proj_norm.getDistance(WorldCoordinates(0, 0));
    
    return FrenetCoordinates(frenet_s, frenet_d);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
WorldCoordinates Map::getXY(const FrenetCoordinates& pos) const
{
    int prev_wp = -1;
    
    while(pos.getS() > waypoints[prev_wp+1].getFrenet().getS() &&
          (prev_wp < (int)(waypoints.size()-1) ))
    {
        prev_wp++;
    }
    
    const int wp2 = (prev_wp+1) % waypoints.size();
    
    const WorldCoordinates prev_waypoints = waypoints[prev_wp].getWorldCoordinats();
    const WorldCoordinates waypoints2 = waypoints[wp2].getWorldCoordinats();
    
    double heading = prev_waypoints.getHeading(waypoints2);
    
    // the x,y,s along the segment
    double seg_s = (pos.getS() - waypoints[prev_wp].getFrenet().getS());
    WorldCoordinates seg = prev_waypoints + WorldCoordinates(seg_s*cos(heading), seg_s*sin(heading));
    
    double perp_heading = heading - M_PI/2;
    
    return seg + WorldCoordinates(pos.getD() * cos(perp_heading), pos.getD() * sin(perp_heading));
}
